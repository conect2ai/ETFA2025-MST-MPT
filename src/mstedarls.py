import numpy as np
# Classe MSTEDARLS
class MSTEDARLS:
    def __init__(self, threshold=4.0, rls_mu=1.0, rls_delta=1000.0, w_init=0.0, n_features=1, correct_outlier=False):
        self.threshold = threshold
        self.rls_mu = rls_mu
        self.rls_delta = rls_delta
        self.correct_outlier = correct_outlier
        self.n_features = n_features
        self.n = np.zeros(n_features)
        self.mean = np.zeros(n_features)
        self.var = np.zeros(n_features)
        self.w = np.full(n_features, w_init, dtype=float)
        self.P = np.array([np.eye(1) * rls_delta for _ in range(n_features)])

    def _teda_outlier(self, x, i):
        self.n[i] += 1
        delta = x - self.mean[i]
        self.mean[i] += delta / self.n[i]
        self.var[i] += delta * (x - self.mean[i])
        if self.n[i] < 2: return False
        sigma2 = self.var[i] / (self.n[i] - 1)
        if sigma2 == 0: return False
        d2 = (x - self.mean[i]) ** 2 / sigma2
        return d2 > self.threshold

    def _rls_predict(self, i):
        return self.w[i]

    def _rls_update(self, x, i):
        phi = np.array([[1.0]])
        y = x
        P = self.P[i]
        w = self.w[i]
        k = P @ phi / (self.rls_mu + phi.T @ P @ phi)
        err = y - (phi.T @ [[w]])[0][0]
        w_new = w + (k.flatten() * err)[0]
        P_new = (P - k @ phi.T @ P) / self.rls_mu
        self.w[i] = w_new
        self.P[i] = P_new

    def update(self, x_vec):
        x_vec = np.asarray(x_vec)
        assert x_vec.shape[0] == self.n_features, "Dimensão incorreta"
        is_outlier = np.zeros(self.n_features, dtype=bool)
        x_corrected = x_vec.copy()
        for i in range(self.n_features):
            outlier = self._teda_outlier(x_vec[i], i)
            is_outlier[i] = outlier
            if outlier and self.correct_outlier:
                x_corrected[i] = self._rls_predict(i)
            self._rls_update(x_corrected[i], i)
        return x_corrected, is_outlier