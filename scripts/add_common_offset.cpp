#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <map>
#include <stdexcept>

class OffsetAdder
{
public:
    OffsetAdder(const std::string& yaml_path, const std::string& output_yaml_path, const std::pair<float, float>& common_reference_offset);
    void addCommonOffset();

private:
    std::string yaml_path_;
    std::string output_yaml_path_;
    std::pair<float, float> common_reference_offset_;
    std::map<int, std::pair<float, float>> offsets_;

    void loadYAML(const std::string& yaml_path);
    void saveOffsetsToYAML(const std::string& output_yaml_path);
};

OffsetAdder::OffsetAdder(const std::string& yaml_path, const std::string& output_yaml_path, const std::pair<float, float>& common_reference_offset)
    : yaml_path_(yaml_path), output_yaml_path_(output_yaml_path), common_reference_offset_(common_reference_offset)
{
}

void OffsetAdder::loadYAML(const std::string& yaml_path)
{
    YAML::Node yaml = YAML::LoadFile(yaml_path);
    for (YAML::const_iterator it = yaml.begin(); it != yaml.end(); ++it) {
        int id = it->first.as<int>();
        std::vector<float> offset = it->second.as<std::vector<float>>();
        offsets_[id] = std::make_pair(offset[0], offset[1]);
    }
}

void OffsetAdder::saveOffsetsToYAML(const std::string& output_yaml_path)
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

void OffsetAdder::addCommonOffset()
{
    loadYAML(yaml_path_);

    for (auto& offset : offsets_) {
        offset.second.first += common_reference_offset_.first;
        offset.second.second += common_reference_offset_.second;
    }

    saveOffsetsToYAML(output_yaml_path_);
}

int main(int argc, char** argv)
{
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <input_yaml> <output_yaml> <common_offset_x> <common_offset_y>" << std::endl;
        return -1;
    }

    std::string input_yaml_path = argv[1];
    std::string output_yaml_path = argv[2];
    float common_offset_x = std::stof(argv[3]);
    float common_offset_y = std::stof(argv[4]);

    std::pair<float, float> common_reference_offset = std::make_pair(common_offset_x, common_offset_y);

    try {
        OffsetAdder adder(input_yaml_path, output_yaml_path, common_reference_offset);
        adder.addCommonOffset();
        std::cout << "Updated offsets saved to " << output_yaml_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}