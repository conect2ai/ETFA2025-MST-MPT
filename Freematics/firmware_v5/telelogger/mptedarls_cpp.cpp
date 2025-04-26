#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

class MPTEDARLS {
public:

    // Resultado de run()
    struct RunResult {
        int outlier_flag;
        std::vector<double> y_pred;
        std::vector<double> x_filtered;
    };

    // Construtor: parâmetros originais do Python com valores default
    MPTEDARLS(
        double threshold = 6.0,
        int rls_n = 2,
        double rls_mu = 0.9999,
        double rls_delta = 0.1,
        const std::vector<double>& w_init = {},
        bool correct_outlier = true,
        int window_size = 0,
        int window_outlier_limit = 15,
        bool use_per_dim_teda = false,
        double ecc_div = 4.0,
        double epsilon = 1e-6,
        bool clip_output = true,
        bool clip_weights = true,
        std::pair<double, double> output_clip_range = {-100.0, 100.0},
        std::pair<double, double> weight_clip_range = {-100.0, 100.0},
        double max_dw = 5.0,
        bool verbose = false)
        : threshold(threshold),
          rls_n(rls_n),
          rls_mu(rls_mu),
          rls_delta(rls_delta),
          correct_outlier(correct_outlier),
          window_size(window_size),
          window_outlier_limit(window_outlier_limit),
          use_per_dim_teda(use_per_dim_teda),
          ecc_div(ecc_div),
          epsilon(epsilon),
          clip_output(clip_output),
          clip_weights(clip_weights),
          output_clip_range(output_clip_range),
          weight_clip_range(weight_clip_range),
          max_dw(max_dw),
          verbose(verbose),
          k(1),
          consecutive_outliers(0)
    {
        mean.assign(rls_n, 0.0);
        var.assign(rls_n, 0.0);
        initRLSEstimates(w_init);
    }

    // Inicializa W e P de acordo com w_init
    void initRLSEstimates(const std::vector<double>& w_init) {
        W.clear();
        P.clear();
        int dim  = rls_n;
        int size = rls_n - 1;
    
        for (int i = 0; i < dim; ++i) {
            // --- coeficientes RLS (vetor) ---
            std::vector<double> w_i(size, 0.0);
            if (!w_init.empty() && (int)w_init.size() == dim) {
                w_i.assign(size, w_init[i]);
            }
            W.push_back(w_i);
    
            // --- matriz P_i = (1/rls_delta) * I(size) ---
            std::vector<std::vector<double>> P_i(size, std::vector<double>(size, 0.0));
            for (int j = 0; j < size; ++j) {
                P_i[j][j] = 1.0 / rls_delta;
            }
            P.push_back(P_i);  // agora P é 3D, compatível com P_i
        }
    }

    /// Redefine o estado do TEDA (média, variância e contador k)
    void resetTeda() {
        k = 1;
        // média e variância zeradas em todas as dimensões
        mean.assign(rls_n, 0.0);
        var .assign(rls_n, 0.0);
        // zera o contador de outliers consecutivos
        consecutive_outliers = 0;
    }

    /**
     * Redefine o estado do RLS.
     * @param W_init Vetores de coeficientes iniciais (mesmo formato de W)
     */
    void resetRLS(const std::vector<std::vector<double>>& W_init) {
        // Substitui W pelos coeficientes fornecidos
        W = W_init;

        // Reconstrói P como (1/rls_delta) * I para cada dimensão
        int dim  = rls_n;
        int size = rls_n - 1;

        P.clear();
        P.reserve(dim);
        for (int i = 0; i < dim; ++i) {
            std::vector<std::vector<double>> P_i(size, std::vector<double>(size, 0.0));
            for (int j = 0; j < size; ++j) {
                P_i[j][j] = 1.0 / rls_delta;
            }
            P.push_back(std::move(P_i));
        }
    }

    bool tedaOutlierGlobal(const std::vector<double>& x) {
        // 1) Calcula delta e atualiza média
        std::vector<double> delta(rls_n);
        for (int i = 0; i < rls_n; ++i) {
            delta[i] = x[i] - mean[i];
            mean[i] += delta[i] / k;
        }

        // 2) Distância ao quadrado
        double dist_sq = std::inner_product(delta.begin(), delta.end(), delta.begin(), 0.0);

        // 3) Atualiza variância global em var[0]
        if (k > 1) {
            var[0] += dist_sq / (k - 1);
        }
        double sigma2 = (k > 1 ? var[0] / (k - 1) : 1e-8);

        // 4) Calcula 'ecc_norm' e compara com limiar
        double ecc      = (1.0 / k) + dist_sq / (k * std::max(sigma2, epsilon));
        double ecc_norm = ecc / ecc_div;
        double thresh   = (threshold * threshold + 1.0) / (2.0 * k);

        return ecc_norm > thresh;
    }

    /// Versão C++ de _rls_predict_all(self, x)
    std::vector<double> rlsPredictAll(const std::vector<double>& x) {
        std::vector<double> y(rls_n);
        int size = rls_n - 1;

        for (int i = 0; i < rls_n; ++i) {
            // monta x_ sem a i-ésima dimensão
            std::vector<double> x_(size);
            for (int j = 0, idx = 0; j < rls_n; ++j) {
                if (j == i) continue;
                x_[idx++] = x[j];
            }

            // predição linear
            double yi = std::inner_product(W[i].begin(), W[i].end(), x_.begin(), 0.0);

            // clipping de saída, se requerido
            if (clip_output) {
                yi = std::max(output_clip_range.first,
                              std::min(yi, output_clip_range.second));
            }
            y[i] = yi;
        }
        return y;
    }

    /// Versão C++ de _rls_update_all(self, d, x)
    void rlsUpdateAll(const std::vector<double>& d, const std::vector<double>& x) {
        int size = rls_n - 1;

        for (int i = 0; i < rls_n; ++i) {
            // monta x_ sem a i-ésima dimensão
            std::vector<double> x_(size);
            for (int j = 0, idx = 0; j < rls_n; ++j) {
                if (j == i) continue;
                x_[idx++] = x[j];
            }

            // erro de predição
            double yi = std::inner_product(W[i].begin(), W[i].end(), x_.begin(), 0.0);
            double ei = d[i] - yi;

            // P[i] * x_
            std::vector<double> Pxi(size, 0.0);
            for (int r = 0; r < size; ++r) {
                for (int c = 0; c < size; ++c) {
                    Pxi[r] += P[i][r][c] * x_[c];
                }
            }

            // ganho g_i
            double denom = rls_mu + std::inner_product(x_.begin(), x_.end(), Pxi.begin(), 0.0);
            denom = std::max(denom, epsilon);
            std::vector<double> gi(size);
            for (int j = 0; j < size; ++j) {
                gi[j] = Pxi[j] / denom;
            }

            // atualização incremental dos pesos
            std::vector<double> dw_i(size);
            for (int j = 0; j < size; ++j) {
                dw_i[j] = gi[j] * ei;
            }

            // limita variação de peso
            double dw_norm = std::sqrt(std::inner_product(dw_i.begin(), dw_i.end(), dw_i.begin(), 0.0));
            if (dw_norm > max_dw) {
                double scale = max_dw / dw_norm;
                for (double &v : dw_i) v *= scale;
            }

            // atualiza P[i] ← (1/rls_mu)·[P[i] – (g_i ⊗ x_)·P[i]]
            // onde (g_i ⊗ x_) é o produto externo
            std::vector<std::vector<double>> temp(size, std::vector<double>(size, 0.0));
            for (int r = 0; r < size; ++r) {
                for (int c = 0; c < size; ++c) {
                    // soma sobre m: g_i[r] * x_[m] * P[i][m][c]
                    double sum = 0.0;
                    for (int m = 0; m < size; ++m) {
                        sum += gi[r] * x_[m] * P[i][m][c];
                    }
                    temp[r][c] = sum;
                }
            }
            for (int r = 0; r < size; ++r) {
                for (int c = 0; c < size; ++c) {
                    P[i][r][c] = (1.0 / rls_mu) * (P[i][r][c] - temp[r][c]);
                }
            }

            // aplica dw_i nos pesos
            for (int j = 0; j < size; ++j) {
                W[i][j] += dw_i[j];
                if (clip_weights) {
                    W[i][j] = std::max(weight_clip_range.first,
                                      std::min(W[i][j], weight_clip_range.second));
                }
            }
        }
    }

    std::vector<bool> tedaOutlierPerDim(const std::vector<double>& x) {
        std::vector<bool> outlier_mask(rls_n, false);
        for (int i = 0; i < rls_n; ++i) {
            // Atualiza média e var por dimensão
            double delta = x[i] - mean[i];
            mean[i] += delta / k;
            var[i] += delta * (x[i] - mean[i]);

            // Só detecta outlier se houver amostras suficientes
            if (k < 2) continue;
            double sigma2 = var[i] / (k - 1);
            if (sigma2 < epsilon) continue;

            // Distância normalizada
            double d2       = (x[i] - mean[i]) * (x[i] - mean[i]) / sigma2;
            double ecc      = (1.0 / k) + d2 / k;
            double ecc_norm = ecc / ecc_div;
            double thresh   = (threshold * threshold + 1.0) / (2.0 * k);

            if (ecc_norm > thresh) {
                outlier_mask[i] = true;
            }
        }
        return outlier_mask;
    }

    RunResult run(const std::vector<double>& x) {
        if ((int)x.size() != rls_n) {
            throw std::invalid_argument("Dimensão de entrada incompatível.");
        }

        int outlier_flag;
        std::vector<double> y_pred(rls_n), x_filtered(rls_n);
        std::vector<bool> mask;

        if (k == 1) {
            // primeira amostra
            mean = x;
            var.assign(rls_n, 0.0);
            outlier_flag = 0;
            y_pred = rlsPredictAll(x);
            x_filtered = x;
        } else {
            // detecção de outlier
            if (use_per_dim_teda) {
                mask = tedaOutlierPerDim(x);
                outlier_flag = std::any_of(mask.begin(), mask.end(), [](bool v){return v;}) ? 1 : 0;
            } else {
                outlier_flag = tedaOutlierGlobal(x) ? 1 : 0;
            }
            // contador de outliers consecutivos
            if (outlier_flag) ++consecutive_outliers; else consecutive_outliers = 0;
            // reset se exceder
            if (window_outlier_limit > 0 && consecutive_outliers >= window_outlier_limit) {
                if (verbose) std::cout << "Reset automático em k=" << k << " por excesso de outliers consecutivos\n";
                resetTeda();
                resetRLS(W);
                consecutive_outliers = 0;
            }
            // predição e correção
            y_pred = rlsPredictAll(x);
            if (outlier_flag && correct_outlier && use_per_dim_teda) {
                x_filtered = x;
                for (int i = 0; i < rls_n; ++i) {
                    if (mask[i]) x_filtered[i] = y_pred[i];
                }
            } else {
                x_filtered = (outlier_flag && correct_outlier) ? y_pred : x;
            }
            // atualização RLS
            rlsUpdateAll(x_filtered, x);
        }
        // armazena histórico
        outlier_flags.push_back(outlier_flag);
        predictions.push_back(y_pred);
        filtered_data.push_back(x_filtered);
        // debug_log opcional (não implementado aqui)

        ++k;

        // ========================
        // DEBUG LOG (linha CSV)
        // ========================
        static std::ofstream debug_file("debug_log_cpp.csv");
        debug_file << std::fixed << std::setprecision(16);
        if (k == 2) {
            debug_file << "k";
            for (int i = 0; i < rls_n; ++i) debug_file << ",x" << i;
            for (int i = 0; i < rls_n; ++i) debug_file << ",mean" << i;
            debug_file << ",var_global,outlier_flag";
            for (int i = 0; i < rls_n; ++i) debug_file << ",y_pred" << i;
            for (int i = 0; i < rls_n; ++i) debug_file << ",x_corr" << i;
            debug_file << "\n";
        }

        // x normalizado
        debug_file << k;
        for (int i = 0; i < rls_n; ++i) debug_file << "," << x[i];

        // média atual
        for (int i = 0; i < rls_n; ++i) debug_file << "," << mean[i];

        // variância global
        debug_file << "," << var[0];

        // flag de outlier
        debug_file << "," << outlier_flag;

        // predições
        for (int i = 0; i < rls_n; ++i) debug_file << "," << y_pred[i];

        // correção aplicada
        for (int i = 0; i < rls_n; ++i) debug_file << "," << x_filtered[i];

        debug_file << "\n";
        return {outlier_flag, y_pred, x_filtered};
    }

private:
    // parâmetros
    double threshold;
    int rls_n;
    double rls_mu;
    double rls_delta;
    bool correct_outlier;
    int window_size;
    int window_outlier_limit;
    bool use_per_dim_teda;
    double ecc_div;
    double epsilon;
    bool clip_output;
    bool clip_weights;
    std::pair<double,double> output_clip_range;
    std::pair<double,double> weight_clip_range;
    double max_dw;
    bool verbose;

    // estado interno
    int k = 1;
    int consecutive_outliers = 0;
    std::vector<double> mean;
    std::vector<double> var;
    std::vector<std::vector<double>> W;
    std::vector<std::vector<std::vector<double>>> P;

    // histórico
    std::vector<int>                            outlier_flags;
    std::vector<std::vector<double>>            predictions;
    std::vector<std::vector<double>>            filtered_data;

};