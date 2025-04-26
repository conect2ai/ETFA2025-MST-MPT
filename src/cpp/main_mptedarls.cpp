#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include "mptedarls_cpp.cpp"

// Para compilar: g++ -std=c++17 -o main_mptedarls main_mptedarls.cpp mptedarls_cpp.cpp

std::vector<std::vector<double>> load_csv(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<std::vector<double>> data;
    std::string line;

    if (!file.is_open()) throw std::runtime_error("Erro ao abrir arquivo: " + filename);
    std::getline(file, line); // Ignora cabeçalho

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::istringstream ss(line);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try {
                row.push_back(std::stod(token));
            } catch (...) {
                row.push_back(0.0);
            }
        }
        if (!row.empty()) data.push_back(row);
    }
    return data;
}

void salvar_csv(const std::string& filename,
    const std::vector<std::vector<double>>& dados,
    const std::vector<std::vector<double>>& y_preds) {

    std::ofstream out(filename);
    if (!out.is_open()) throw std::runtime_error("Erro ao criar: " + filename);
    out << std::fixed << std::setprecision(16);

    // Cabeçalho
    out << "speed,rpm,tp,load,timing,"
        << "y_speed,y_rpm,y_tp,y_load,y_timing\n";

    for (size_t i = 0; i < dados.size(); ++i) {
        for (size_t j = 0; j < dados[i].size(); ++j)
            out << dados[i][j] << (j + 1 < dados[i].size() ? "," : ",");

        for (size_t j = 0; j < y_preds[i].size(); ++j)
            out << y_preds[i][j] << (j + 1 < y_preds[i].size() ? "," : "");

        out << "\n";
    }
}

int main() {
    try {
        auto X_original = load_csv("../../data/dados_sem_outliers_fastback.csv");

        if (X_original.empty()) {
            std::cerr << "Erro: arquivo de entrada vazio." << std::endl;
            return 1;
        }

        int n_features = X_original[0].size();
        int n_amostras = X_original.size();

        // Normalização Min-Max
        std::vector<double> min_vals = { 0. ,  0. ,  0. , 0. , -36. };
        std::vector<double> max_vals = { 216.83813265, 6267.89629447,  265.92001187,  280.9220282,   127.99778524 };
        std::vector<std::vector<double>> X_norm;

        for (const auto& row : X_original) {
            std::vector<double> norm_row(row.size());
            for (size_t i = 0; i < row.size(); ++i) {
                norm_row[i] = (row[i] - min_vals[i]) / (max_vals[i] - min_vals[i]);
            }
            X_norm.push_back(norm_row);
        }

        // Modelo com hiperparâmetros ajustados
        MPTEDARLS model(
            5.592,                       // threshold
            n_features,                 // rls_n
            0.9249,                     // rls_mu
            0.1,                        // rls_delta
            std::vector<double>(n_features, 0.0), // w_init
            true,                       // correct_outlier
            0,                          // window_size
            5,                          // window_outlier_limit
            false,                      // use_per_dim_teda
            6.0,                        // ecc_div
            1e-6,                       // epsilon
            true,                       // clip_output
            true,                       // clip_weights
            {-100.0, 100.0},            // output_clip_range
            {-100.0, 100.0},            // weight_clip_range
            10.0,                       // max_dw
            false                       // verbose
        );

        std::vector<std::vector<double>> X_corr, y_preds;

        for (const auto& x : X_norm) {
            auto result = model.run(x);
            X_corr.push_back(result.x_filtered);
            y_preds.push_back(result.y_pred);
        }

        // Desscalar
        for (auto& row : X_corr) {
            for (size_t i = 0; i < row.size(); ++i)
                row[i] = row[i] * (max_vals[i] - min_vals[i]) + min_vals[i];
        }

        for (auto& row : y_preds) {
            for (size_t i = 0; i < row.size(); ++i)
                row[i] = row[i] * (max_vals[i] - min_vals[i]) + min_vals[i];
        }

        salvar_csv("../../data/output_fastback_sem_outlier.csv", X_corr, y_preds);
        std::cout << "Arquivo output_fastback_sem_outlier.csv salvo com sucesso.\n";

        // Diferença para análise de erro posterior (ex: MAE)
        std::ofstream fout_diff("../../data/diff_fastback_sem_outlier.csv");
        fout_diff << "speed_diff,rpm_diff,tp_diff,load_diff,timing_diff\n";
        for (int i = 0; i < n_amostras; ++i) {
            for (int d = 0; d < n_features; ++d) {
                double diff = std::abs(X_corr[i][d] - X_original[i][d]);
                fout_diff << diff;
                if (d + 1 < n_features) fout_diff << ",";
            }
            fout_diff << "\n";
        }
        std::cout << "Arquivo ../../data/diff_fastback_sem_outlier.csv salvo com sucesso.\n";

    } catch (const std::exception& e) {
        std::cerr << "Erro: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}