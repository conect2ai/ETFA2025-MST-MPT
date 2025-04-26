
#include <iostream>
#include "mptedarls.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <fstream>

MPTEDARLS::MPTEDARLS(double threshold, int rls_n, double rls_mu, double rls_delta,
        double w_init, bool correct_outlier, bool use_per_dim_teda,
        double ecc_div, double epsilon, bool clip_output, bool clip_weights,
        double output_clip_min, double output_clip_max,
        double weight_clip_min, double weight_clip_max,
        double max_dw)
    : threshold_(threshold), rls_n_(rls_n), rls_mu_(rls_mu), rls_delta_(rls_delta),
    correct_outlier_(correct_outlier), use_per_dim_teda_(use_per_dim_teda),
    ecc_div_(ecc_div), epsilon_(epsilon), clip_output_(clip_output), clip_weights_(clip_weights),
    output_clip_min_(output_clip_min), output_clip_max_(output_clip_max),
    weight_clip_min_(weight_clip_min), weight_clip_max_(weight_clip_max),
    max_dw_(max_dw), k_(1), consecutive_outliers_(0), window_outlier_limit_(15)
{
    int dim = rls_n_ - 1;
    W_.resize(rls_n_, std::vector<double>(dim, w_init));
    P_.resize(rls_n_, std::vector<std::vector<double>>(dim, std::vector<double>(dim, 0.0)));
    for (int i = 0; i < rls_n_; ++i)
        for (int j = 0; j < dim; ++j)
            P_[i][j][j] = 1.0 / rls_delta_;
    mean_ = std::vector<double>(rls_n_, 0.0);
    var_ = std::vector<double>(rls_n_, 0.0);

    debug_log_.open("debug_cpp.csv");
    debug_log_ << "k";
    for (int i = 0; i < rls_n_; ++i) debug_log_ << ",x" << i;
    for (int i = 0; i < rls_n_; ++i) debug_log_ << ",mean" << i;
    for (int i = 0; i < rls_n_; ++i) debug_log_ << ",y_pred" << i;
    for (int i = 0; i < rls_n_; ++i) debug_log_ << ",x_corr" << i;
    debug_log_ << ",var_global,outlier_flag\n";
}

MPTEDARLS::~MPTEDARLS() {
    if (debug_log_.is_open()) {
        debug_log_.close();
    }
}

void MPTEDARLS::reset_teda() {
    mean_ = std::vector<double>(rls_n_, 0.0);
    var_ = std::vector<double>(rls_n_, 0.0);
    k_ = 1;
}

void MPTEDARLS::reset_rls() {
    int dim = rls_n_ - 1;
    for (int i = 0; i < rls_n_; ++i) {
        W_[i] = std::vector<double>(dim, 0.0);
        for (int j = 0; j < dim; ++j)
            for (int k = 0; k < dim; ++k)
                P_[i][j][k] = (j == k) ? (1.0 / rls_delta_) : 0.0;
    }
}

void MPTEDARLS::update(std::vector<double>& x_corr, std::vector<double>& y_pred, bool& outlier_flag) {
    if ((int)x_corr.size() != rls_n_) {
        throw std::runtime_error("Dimensão incorreta da entrada.");
    }

    std::vector<double> x = x_corr; // cópia da entrada original
    y_pred = rls_predict_all(x);    // predição antes de qualquer modificação
    std::vector<bool> outlier_mask;

    if (k_ == 1) {
        mean_ = x;
        var_ = std::vector<double>(rls_n_, 0.0);
        outlier_flag = false;
    } else {
        if (use_per_dim_teda_) {
            outlier_mask = teda_outlier_per_dim(x, k_);
            outlier_flag = std::any_of(outlier_mask.begin(), outlier_mask.end(), [](bool b) { return b; });
        } else {
            outlier_flag = teda_outlier(x);
        }

        if (outlier_flag) {
            consecutive_outliers_++;
        } else {
            consecutive_outliers_ = 0;
        }

        if (window_outlier_limit_ > 0 && consecutive_outliers_ >= window_outlier_limit_) {
            reset_teda();
            reset_rls();
            consecutive_outliers_ = 0;
        }

        if (outlier_flag && correct_outlier_) {
            if (use_per_dim_teda_) {
                for (int i = 0; i < rls_n_; ++i)
                    if (outlier_mask[i])
                        x_corr[i] = y_pred[i];  // corrige somente onde for outlier
            } else {
                x_corr = y_pred; // correção global
            }
        } else {
            x_corr = x; // mantém original
        }
    }

    // Atualiza pesos RLS com valor corrigido ou original
    rls_update_all(x_corr, x);

    // Armazena histórico
    outlier_flags_.push_back(outlier_flag ? 1 : 0);
    predictions_.push_back(y_pred);
    filtered_data_.push_back(x_corr);

    debug_log_ << k_;
    for (int i = 0; i < rls_n_; ++i) debug_log_ << "," << x[i];
    for (int i = 0; i < rls_n_; ++i) debug_log_ << "," << mean_[i];
    for (int i = 0; i < rls_n_; ++i) debug_log_ << "," << y_pred[i];
    for (int i = 0; i < rls_n_; ++i) debug_log_ << "," << x_corr[i];
    debug_log_ << "," << var_[0] << "," << (outlier_flag ? 1 : 0) << "\n";

    k_++;
}


std::vector<double> MPTEDARLS::rls_predict_all(const std::vector<double>& x) {
    std::vector<double> y_pred(rls_n_, 0.0);

    for (int i = 0; i < rls_n_; ++i) {
        // Construir entrada x_ removendo x[i]
        std::vector<double> x_;
        for (int j = 0; j < rls_n_; ++j) {
            if (j != i) x_.push_back(x[j]);
        }

        // Produto escalar W[i] * x_
        double y = 0.0;
        for (int j = 0; j < x_.size(); ++j) {
            y += W_[i][j] * x_[j];
        }

        if (clip_output_) {
            y = std::min(std::max(y, output_clip_min_), output_clip_max_);
        }

        y_pred[i] = y;
    }

    return y_pred;
}

void MPTEDARLS::rls_update_all(const std::vector<double>& d, const std::vector<double>& x) {
    for (int i = 0; i < rls_n_; ++i) {
        // x_ = x sem o elemento i
        std::vector<double> x_;
        for (int j = 0; j < rls_n_; ++j) {
            if (j != i) x_.push_back(x[j]);
        }

        std::vector<double>& w = W_[i];
        std::vector<std::vector<double>>& P = P_[i];

        // Px = P * x_
        std::vector<double> Px(x_.size(), 0.0);
        for (int j = 0; j < x_.size(); ++j)
            for (int k = 0; k < x_.size(); ++k)
                Px[j] += P[j][k] * x_[k];

        double denom = rls_mu_;
        for (int j = 0; j < x_.size(); ++j)
            denom += x_[j] * Px[j];

        std::vector<double> K(x_.size(), 0.0);
        for (int j = 0; j < x_.size(); ++j)
            K[j] = Px[j] / denom;

        double y = 0.0;
        for (int j = 0; j < x_.size(); ++j)
            y += w[j] * x_[j];
        double e = d[i] - y;

        for (int j = 0; j < x_.size(); ++j) {
            double dw = K[j] * e;
            if (std::abs(dw) > max_dw_) {
                dw = (dw > 0 ? 1 : -1) * max_dw_;
            }
            w[j] += dw;
            if (clip_weights_) {
                w[j] = std::min(std::max(w[j], weight_clip_min_), weight_clip_max_);
            }
        }

        for (int j = 0; j < x_.size(); ++j)
            for (int k = 0; k < x_.size(); ++k)
                P[j][k] = (P[j][k] - K[j] * x_[k] * denom) / rls_mu_;
    }
}

bool MPTEDARLS::teda_outlier(const std::vector<double>& x) {
    // Calcula delta = x - mean
    std::vector<double> delta(rls_n_);
    for (int i = 0; i < rls_n_; ++i) {
        delta[i] = x[i] - mean_[i];
    }

    // Atualiza média
    for (int i = 0; i < rls_n_; ++i) {
        mean_[i] += delta[i] / k_;
    }

    // Distância quadrada ||x - mean||²
    double dist_sq = 0.0;
    for (int i = 0; i < rls_n_; ++i) {
        dist_sq += delta[i] * delta[i];
    }

    // Atualiza variância acumulada global (usar var_[0])
    if (k_ > 1) {
        var_[0] += dist_sq / (k_ - 1);
    }

    double sigma2 = (k_ > 1) ? var_[0] / (k_ - 1) : 1e-8;
    sigma2 = std::max(sigma2, epsilon_);

    // Calcula ECC normalizado
    double ecc = (1.0 / k_) + dist_sq / (k_ * sigma2);
    double ecc_norm = ecc / ecc_div_;

    // Threshold TEDA
    double threshold_val = (threshold_ * threshold_ + 1.0) / (2.0 * k_);
    return ecc_norm > threshold_val;
}

std::vector<bool> MPTEDARLS::teda_outlier_per_dim(const std::vector<double>& x, int index) {
    std::vector<bool> flags(rls_n_, false);

    for (int i = 0; i < rls_n_; ++i) {
        double delta = x[i] - mean_[i];
        mean_[i] += delta / k_;
        var_[i] += delta * (x[i] - mean_[i]);

        if (k_ < 2) continue;

        double sigma2 = var_[i] / (k_ - 1);
        if (sigma2 < epsilon_) continue;

        double d2 = (x[i] - mean_[i]) * (x[i] - mean_[i]) / sigma2;
        double ecc = (1.0 / k_) + d2 / k_;
        double ecc_norm = ecc / ecc_div_;
        double threshold_val = (threshold_ * threshold_ + 1.0) / (2.0 * k_);

        if (ecc_norm > threshold_val) {
            flags[i] = true;
        }

        // (Opcional) Debug para índice específico
        if (index == 410) {
            std::cout << "Variável " << i
                      << " | x = " << x[i]
                      << ", média = " << mean_[i]
                      << ", var = " << var_[i]
                      << ", sigma² = " << sigma2
                      << ", d² = " << d2
                      << ", ecc_norm = " << ecc_norm
                      << ", threshold = " << threshold_val
                      << ", flag = " << flags[i] << "\n";
        }
    }

    if (index == 410) {
        std::cout << "[FIM DEBUG TEDA - INDEX 410]\n";
    }

    return flags;
}