import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix
from sklearn.preprocessing import MinMaxScaler  # Para normalização opcional

# ==========================
# 1. Nova classe com melhorias estruturais e controle de explosões
# ==========================
class MPTEDARLS:
    def __init__(
        self,
        threshold=2.0,
        rls_n=2,
        rls_mu=0.99,         # <-- Ajustado de 0.9999 para 0.99
        rls_delta=0.1,
        w_init=None,
        correct_outlier=True,
        window_size=None,
        window_outlier_limit=None,
        use_per_dim_teda=False,
        ecc_div=2.0,
        epsilon=1e-6,
        clip_output=True,
        clip_weights=True,
        output_clip_range=(-100, 100),
        weight_clip_range=(-100, 100),
        max_dw=50.0,
        verbose=False
    ):
        self.threshold = threshold
        self.rls_n = rls_n
        self.rls_mu = rls_mu
        self.rls_delta = rls_delta
        self.correct_outlier = correct_outlier
        self.window_size = window_size
        self.window_outlier_limit = window_outlier_limit
        self.use_per_dim_teda = use_per_dim_teda
        self.ecc_div = ecc_div
        self.epsilon = epsilon
        self.clip_output = clip_output
        self.clip_weights = clip_weights
        self.output_clip_range = output_clip_range
        self.weight_clip_range = weight_clip_range
        self.max_dw = max_dw
        self.verbose = verbose

        self._reset_teda()
        self._init_rls(w_init)

        self.outlier_flags = []
        self.predictions   = []
        self.filtered_data = []
        self.consecutive_outliers = 0
        self.debug_log = []

    def _reset_teda(self):
        self.k = 1
        self.mean = np.zeros(self.rls_n)  # Inicializa a média como vetor de zeros
        # Se use_per_dim_teda=True, cada dimensão mantém sua variância separada
        self.var = np.zeros(self.rls_n)

    def _init_rls(self, w_init):
        self.W = []
        self.P = []
        for i in range(self.rls_n):
            size = self.rls_n - 1
            if isinstance(w_init, list) and len(w_init) == self.rls_n:
                w_i = np.array(w_init[i], dtype=float)
            elif w_init is None or np.isscalar(w_init):
                w_i = np.full(size, w_init if w_init is not None else 0.0)
            else:
                raise ValueError("w_init deve ser escalar, None ou lista de vetores.")
            self.W.append(w_i)
            self.P.append((1.0 / self.rls_delta) * np.eye(size))

    def _reset_rls(self, w_init):
        if isinstance(w_init, list) and all(isinstance(w, np.ndarray) for w in w_init):
            self._init_rls(w_init)
        else:
            self._init_rls([np.full(self.rls_n - 1, w_init if w_init is not None else 0.0) for _ in range(self.rls_n)])

    def _teda_outlier_per_dim(self, x):
        """
        Atualiza mean e var por dimensão e retorna máscara booleana
        indicando se cada dimensão é outlier individualmente.
        """
        outlier_mask = np.zeros(self.rls_n, dtype=bool)
        for i in range(self.rls_n):
            delta = x[i] - self.mean[i]
            self.mean[i] += delta / self.k
            self.var[i] += delta * (x[i] - self.mean[i])

            if self.k < 2:
                continue
            sigma2 = self.var[i] / (self.k - 1)
            if sigma2 < self.epsilon:
                continue
            d2 = (x[i] - self.mean[i])**2 / sigma2
            ecc = (1.0 / self.k) + d2 / self.k
            ecc_norm = ecc / self.ecc_div

            threshold = (self.threshold**2 + 1.0) / (2.0 * self.k)
            if ecc_norm > threshold:
                outlier_mask[i] = True
        return outlier_mask

    def _teda_outlier_global(self, x):
        """
        Atualiza mean e var de forma global (único valor de variância
        armazenado em self.var[0]) e retorna True/False se x é outlier.
        """
        delta = x - self.mean
        self.mean += delta / self.k

        dist_sq = np.dot(delta, delta)
        if self.k > 1:
            self.var[0] += dist_sq / (self.k - 1)

        sigma2 = self.var[0] / (self.k - 1) if self.k > 1 else 1e-8
        ecc = (1.0 / self.k) + dist_sq / (self.k * max(sigma2, self.epsilon))
        ecc_norm = ecc / self.ecc_div
        threshold = (self.threshold**2 + 1.0) / (2.0 * self.k)
        return ecc_norm > threshold

    def _rls_predict_all(self, x):
        """
        Prediz cada dimensão i usando as demais (x sem x[i]).
        """
        y = np.zeros(self.rls_n)
        for i in range(self.rls_n):
            x_ = np.delete(x, i)
            y[i] = self.W[i] @ x_
            if self.clip_output:
                y[i] = np.clip(y[i], *self.output_clip_range)
        return y

    def _rls_update_all(self, d, x):
        """
        Atualiza todos os filtros RLS, um para cada dimensão.
        d = vetor alvo (valor correto para cada dimensão)
        x = entrada original
        """
        for i in range(self.rls_n):
            x_ = np.delete(x, i)
            y_i = self.W[i] @ x_
            e_i = d[i] - y_i

            denom = self.rls_mu + x_ @ (self.P[i] @ x_)
            denom = max(denom, self.epsilon)  # Evitar divisão por zero
            g_i = (self.P[i] @ x_) / denom

            dw_i = g_i * e_i
            dw_norm = np.linalg.norm(dw_i)
            if dw_norm > self.max_dw:
                dw_i *= (self.max_dw / dw_norm)

            self.P[i] = (1.0 / self.rls_mu) * (self.P[i] - np.outer(g_i, x_) @ self.P[i])
            self.W[i] += dw_i

            if self.clip_weights:
                self.W[i] = np.clip(self.W[i], *self.weight_clip_range)

    def run(self, x):
        """
        Roda uma amostra (x) pelo TEDA (para detecção de outlier) 
        e pelo RLS (para predição/atualização de pesos).
        """
        x = np.asarray(x, dtype=float)
        if x.shape[0] != self.rls_n:
            raise ValueError("Dimensão de entrada incompatível.")

        if self.k == 1:
            # Primeira amostra: inicializa mean e var
            self.mean = x.copy()
            self.var = np.zeros(self.rls_n)
            outlier_flag = 0
            y_pred = self._rls_predict_all(x)
            x_filtered = x.copy()
        else:
            # Verifica outlier
            if self.use_per_dim_teda:
                outlier_mask = self._teda_outlier_per_dim(x)
                outlier_flag = int(outlier_mask.any())
            else:
                outlier_flag = int(self._teda_outlier_global(x))

            # Conta outliers consecutivos para eventual reset
            if outlier_flag:
                self.consecutive_outliers += 1
            else:
                self.consecutive_outliers = 0

            if self.window_outlier_limit and self.consecutive_outliers >= self.window_outlier_limit:
                if self.verbose:
                    print(f"Reset automático em k={self.k} por excesso de outliers consecutivos")
                self._reset_teda()
                self._reset_rls([w.copy() for w in self.W])
                self.consecutive_outliers = 0

            # Predição RLS
            y_pred = self._rls_predict_all(x)

            # === PRINCIPAL MUDANÇA AQUI: corrigir apenas dimensões outliers ===
            if outlier_flag and self.correct_outlier and self.use_per_dim_teda:
                # Corrige somente dimensões apontadas como outlier
                x_filtered = x.copy()
                for i_dim in range(self.rls_n):
                    if outlier_mask[i_dim]:
                        x_filtered[i_dim] = y_pred[i_dim]
            else:
                # Para o caso de outlier "global" ou se use_per_dim_teda=False
                x_filtered = y_pred if (outlier_flag and self.correct_outlier) else x.copy()

            # Atualiza RLS com o valor "corrigido" ou original
            self._rls_update_all(d=x_filtered, x=x)

        self.outlier_flags.append(outlier_flag)
        self.predictions.append(y_pred)
        self.filtered_data.append(x_filtered)
        self.k += 1

        self.debug_log.append({
            "k": self.k,
            **{f"x{i}": x[i] for i in range(self.rls_n)},
            **{f"mean{i}": self.mean[i] for i in range(self.rls_n)},
            "var_global": self.var[0] if not self.use_per_dim_teda else -1,
            "outlier_flag": outlier_flag,
            **{f"y_pred{i}": y_pred[i] for i in range(self.rls_n)},
            **{f"x_corr{i}": x_filtered[i] for i in range(self.rls_n)},
        })

        return outlier_flag, y_pred, x_filtered