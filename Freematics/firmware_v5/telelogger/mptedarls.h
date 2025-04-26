
#ifndef MPTEDARLS_H
#define MPTEDARLS_H

#include <vector>
#include <utility>
#include <fstream>

class MPTEDARLS {
public:
    MPTEDARLS(double threshold = 2.0, int rls_n = 2, double rls_mu = 0.99, double rls_delta = 0.1,
              double w_init = 0.0, bool correct_outlier = true, bool use_per_dim_teda = false,
              double ecc_div = 2.0, double epsilon = 1e-6, bool clip_output = true, bool clip_weights = true,
              double output_clip_min = -300, double output_clip_max = 300,
              double weight_clip_min = -100, double weight_clip_max = 100,
              double max_dw = 50.0);

    ~MPTEDARLS();

    // double update(const std::vector<double>& x);
    void update(std::vector<double>& x_corr, std::vector<double>& y_pred, bool& outlier_flag);

    std::vector<int> getOutlierFlags() const { return outlier_flags_; }
    std::vector<std::vector<double>> getPredictions() const { return predictions_; }

private:
    bool teda_outlier(const std::vector<double>& x);
    std::vector<bool> teda_outlier_per_dim(const std::vector<double>& x, int index = -1);

    void rls_update_all(const std::vector<double>& d, const std::vector<double>& x);
    std::vector<double> rls_predict_all(const std::vector<double>& x);

    void reset_teda();
    void reset_rls();

    double clip(double value, double min_val, double max_val);

    // Configuração
    double threshold_, rls_mu_, rls_delta_;
    bool correct_outlier_, use_per_dim_teda_, clip_output_, clip_weights_;
    double ecc_div_, epsilon_;
    double output_clip_min_, output_clip_max_;
    double weight_clip_min_, weight_clip_max_;
    double max_dw_;
    int rls_n_;

    // Estados
    int k_;
    int consecutive_outliers_;
    int window_outlier_limit_;

    std::vector<std::vector<double>> W_;
    std::vector<std::vector<std::vector<double>>> P_;
    std::vector<double> mean_;
    std::vector<double> var_;

    std::vector<int> outlier_flags_;
    std::vector<std::vector<double>> predictions_;
    std::vector<std::vector<double>> filtered_data_;

    // Inicializa o log de debug
    std::ofstream debug_log_;
};

#endif
