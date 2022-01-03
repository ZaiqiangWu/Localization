#pragma once
#include<chrono>
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

class Timer
{
public:
	Timer()
	{
		t_begin = high_resolution_clock::now();
	}
	float GetTime()
	{
		return (float)std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - t_begin).count() / 1000;
	}
private:
	high_resolution_clock::time_point t_begin;
};