#include "mstedarls.h"
#include <stdexcept>

MSTEDARLS::MSTEDARLS(double threshold, double rls_mu, double rls_delta,
                     double w_init, int n_features, bool correct_outlier)
    : threshold_(threshold), rls_mu_(rls_mu), rls_delta_(rls_delta),
      correct_outlier_(correct_outlier), n_features_(n_features)
{
    n_ = std::vector<double>(n_features_, 0.0);
    mean_ = std::vector<double>(n_features_, 0.0);
    var_ = std::vector<double>(n_features_, 0.0);
    w_ = std::vector<double>(n_features_, w_init);
    P_ = std::vector<double>(n_features_, rls_delta_);
}

bool MSTEDARLS::teda_outlier(double x, int i) {
    n_[i] += 1.0;
    double delta = x - mean_[i];
    mean_[i] += delta / n_[i];
    var_[i] += delta * (x - mean_[i]);

    if (n_[i] < 2.0) return false;

    double sigma2 = var_[i] / (n_[i] - 1.0);
    if (sigma2 == 0.0) return false;

    double d2 = (x - mean_[i]) * (x - mean_[i]) / sigma2;
    return d2 > threshold_;
}

double MSTEDARLS::rls_predict(int i) {
    return w_[i];
}

void MSTEDARLS::rls_update(double x, int i) {
    double phi = 1.0; // entrada univariada
    double P = P_[i];
    double w = w_[i];

    double k = P * phi / (rls_mu_ + phi * P * phi);
    double err = x - (phi * w);
    double w_new = w + k * err;
    double P_new = (P - k * phi * P) / rls_mu_;

    w_[i] = w_new;
    P_[i] = P_new;
}

std::pair<std::vector<double>, std::vector<bool>> MSTEDARLS::update(const std::vector<double>& x_vec) {
    if (x_vec.size() != static_cast<size_t>(n_features_)) {
        throw std::runtime_error("Dimensão da entrada incorreta");
    }

    std::vector<double> x_corrected = x_vec;
    std::vector<bool> is_outlier(n_features_, false);

    for (int i = 0; i < n_features_; ++i) {
        bool outlier = teda_outlier(x_vec[i], i);
        is_outlier[i] = outlier;

        if (outlier && correct_outlier_) {
            x_corrected[i] = rls_predict(i);
        }

        rls_update(x_corrected[i], i);
    }

    return {x_corrected, is_outlier};
}