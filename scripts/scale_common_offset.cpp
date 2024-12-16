#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <map>
#include <stdexcept>

class OffsetScaler
{
public:
    OffsetScaler(const std::string& yaml_path, const std::string& output_yaml_path, float scale_factor_x, float scale_factor_y);
    void scaleOffsets();

private:
    std::string yaml_path_;
    std::string output_yaml_path_;
    float scale_factor_x_;
    float scale_factor_y_;
    std::map<int, std::pair<float, float>> offsets_;

    void loadYAML(const std::string& yaml_path);
    void saveOffsetsToYAML(const std::string& output_yaml_path);
};

OffsetScaler::OffsetScaler(const std::string& yaml_path, const std::string& output_yaml_path, float scale_factor_x, float scale_factor_y)
    : yaml_path_(yaml_path), output_yaml_path_(output_yaml_path), scale_factor_x_(scale_factor_x), scale_factor_y_(scale_factor_y)
{
}

void OffsetScaler::loadYAML(const std::string& yaml_path)
{
    YAML::Node yaml = YAML::LoadFile(yaml_path);
    for (YAML::const_iterator it = yaml.begin(); it != yaml.end(); ++it) {
        int id = it->first.as<int>();
        std::vector<float> offset = it->second.as<std::vector<float>>();
        offsets_[id] = std::make_pair(offset[0], offset[1]);
    }
}

void OffsetScaler::saveOffsetsToYAML(const std::string& output_yaml_path)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& offset : offsets_) {
        out << YAML::Key << offset.first;
        out << YAML::Value << YAML::Flow << YAML::BeginSeq << offset.second.first << offset.second.second << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream fout(output_yaml_path);
    if (!fout.is_open()) {
        throw std::runtime_error("Could not open YAML file for writing: " + output_yaml_path);
    }
    fout << out.c_str();
    fout.close();
}

void OffsetScaler::scaleOffsets()
{
    loadYAML(yaml_path_);

    for (auto& offset : offsets_) {
        offset.second.first *= scale_factor_x_;
        offset.second.second *= scale_factor_y_;
    }

    saveOffsetsToYAML(output_yaml_path_);
}

int main(int argc, char** argv)
{
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <input_yaml> <output_yaml> <scale_factor_x> <scale_factor_y>" << std::endl;
        return -1;
    }

    std::string input_yaml_path = argv[1];
    std::string output_yaml_path = argv[2];
    float scale_factor_x = std::stof(argv[3]);
    float scale_factor_y = std::stof(argv[4]);

    try {
        OffsetScaler scaler(input_yaml_path, output_yaml_path, scale_factor_x, scale_factor_y);
        scaler.scaleOffsets();
        std::cout << "Scaled offsets saved to " << output_yaml_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}