#ifndef MSTEDARLS_H
#define MSTEDARLS_H

#include <vector>
#include <utility>

class MSTEDARLS {
public:
    MSTEDARLS(double threshold = 4.0, double rls_mu = 1.0, double rls_delta = 1000.0,
              double w_init = 0.0, int n_features = 1, bool correct_outlier = false);

    std::pair<std::vector<double>, std::vector<bool>> update(const std::vector<double>& x_vec);

private:
    bool teda_outlier(double x, int i);
    double rls_predict(int i);
    void rls_update(double x, int i);

    double threshold_;
    double rls_mu_;
    double rls_delta_;
    bool correct_outlier_;
    int n_features_;

    std::vector<double> n_;
    std::vector<double> mean_;
    std::vector<double> var_;
    std::vector<double> w_;   // pesos do RLS
    std::vector<double> P_;   // matriz P univariada (1x1 por feature)
};

#endif // MSTEDARLS_H