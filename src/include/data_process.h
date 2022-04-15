#pragma once
#include<vector>
#include<iostream>
using namespace std;
#include "nlohmann/json.hpp"
using json = nlohmann::json;
void json_test();
float get_median(vector<float> datas);
float get_mean(vector<float> datas);

void json_test()
{
    json j;

    // add a number that is stored as double (note the implicit conversion of j to an object)
    j["pi"] = 3.141;

    // add a Boolean that is stored as bool
    j["happy"][0] = true;
    j["happy"][1] = false;

    // add a string that is stored as std::string
    j["name"] = "Niels";

    // add another null object by passing nullptr
    j["nothing"] = nullptr;

    // add an object inside the object
    j["answer"]["everything"] = 42;

    // add an array that is stored as std::vector (using an initializer list)
    vector<float> datas = { 1, 0, 2, 6 };
    j["list"] = datas;
    std::ofstream o("pretty.json");
    o << std::setw(4) << j << std::endl;
}
float get_median(vector<float> datas)
{
    sort(datas.begin(), datas.end());
    int size = datas.size();
    if (size % 2 == 0)
    {
        return (datas[size / 2 - 1] + datas[size / 2]) * 0.5f;
    }
    else
    {
        return datas[size / 2];
    }
}

float get_mean(vector<float> datas)
{
    float result = 0.0f;
    for (int i = 0; i < datas.size(); i++)
    {
        result += datas[i];
    }
    result /= datas.size();
    return result;
}
