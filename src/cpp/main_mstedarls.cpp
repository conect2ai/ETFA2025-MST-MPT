#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "mstedarls.h"

// Para compilar:
// g++ -std=c++17 main_mstedarls.cpp mstedarls.cpp -o mstedarls_wo

int main() {
    std::string input_file = "../../data/dados_sem_outliers_morsinaldo.csv";
    std::string output_file = "../../data/output_morsinaldo_mstedarls.csv";
    std::ifstream file(input_file);

    if (!file.is_open()) {
        std::cerr << "Erro ao abrir o arquivo: " << input_file << std::endl;
        return 1;
    }

    std::vector<std::vector<double>> data;
    std::vector<std::string> column_names;
    std::string line;

    // Leitura do cabeçalho
    if (std::getline(file, line)) {
        std::istringstream header_stream(line);
        std::string col;
        while (std::getline(header_stream, col, ',')) {
            column_names.push_back(col);
        }
    }

    // Leitura dos dados
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(iss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                values.push_back(0.0);
            }
        }

        if (!values.empty()) data.push_back(values);
    }

    file.close();

    if (data.empty()) {
        std::cerr << "Arquivo CSV vazio ou inválido." << std::endl;
        return 1;
    }

    int n_features = data[0].size();

    // Instanciador ajustado com os hiperparâmetros do trim-sweep-10
    MSTEDARLS mstedarls(
        8.414,    // threshold
        0.7,      // rls_mu
        1000.0,   // rls_delta
        1.0,      // w_init
        n_features,
        true      // correct_outlier
    );

    // Criar o arquivo de saída
    std::ofstream out_file(output_file);
    if (!out_file.is_open()) {
        std::cerr << "Erro ao criar arquivo de saída: " << output_file << std::endl;
        return 1;
    }

    // Cabeçalho
    for (const auto& col : column_names) {
        out_file << col << "_corrected,";
    }
    for (size_t i = 0; i < column_names.size(); ++i) {
        out_file << "outlier_" << column_names[i];
        if (i != column_names.size() - 1) out_file << ",";
    }
    out_file << "\n";

    // Processamento
    for (const auto& sample : data) {
        auto [corrected, is_outlier] = mstedarls.update(sample);

        for (double val : corrected)
            out_file << val << ",";
        for (size_t i = 0; i < is_outlier.size(); ++i) {
            out_file << (is_outlier[i] ? "1" : "0");
            if (i != is_outlier.size() - 1) out_file << ",";
        }
        out_file << "\n";
    }

    out_file.close();
    std::cout << "Arquivo processado e salvo em: " << output_file << std::endl;
    return 0;
}