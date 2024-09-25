#ifndef TIME_COUNT_HPP_
#define TIME_COUNT_HPP_

#include <list>
#include <string>
#include <iostream>
#include <chrono>

namespace PowerDiagramGenerator {
	class TimeCount {
	public:
		TimeCount() {
		}
		~TimeCount() {
			(std::list<std::pair<std::string, double>>()).swap(time_pairs_);
			(std::list<double>()).swap(time_duration_);
		}

		void start() {
			start_time_ = std::chrono::system_clock::now();
		}

		void stop() {
			auto cur_time = std::chrono::system_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(cur_time - start_time_);
			double duration_time = double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;

			time_duration_.push_back(duration_time);
		}

		void reset() {
			(std::list<double>()).swap(time_duration_);
		}

		void count_time(const std::string func_name) {
			if (time_duration_.size() == 0) {
				stop();
			}

			double total_time = 0;
			for (auto td : time_duration_) {
				total_time += td;
			}
			time_pairs_.emplace_back(std::make_pair(func_name, total_time));

			reset();
		}

		void cover_count_time(const std::string func_name) {
			if (time_duration_.size() == 0) {
				stop();
			}

			double total_time = 0;
			for (auto td : time_duration_) {
				total_time += td;
			}

			time_pairs_.pop_back();
			time_pairs_.emplace_back(std::make_pair(func_name, total_time));

			reset();
		}

		void print_time_count() {
			double total_time = 0;

			std::cout << "---------- TIME COUNT ----------" << std::endl;
			for (auto tp : time_pairs_) {
				std::cout << tp.first << ":" << std::endl;
				for (int j = 0; j < indent_num_; j++) {
					std::cout << "\t";
				}
				std::cout << std::fixed << "\033[33m" << tp.second << "\033[0m" << ".sec" << std::endl;

				total_time += tp.second;
			}
			std::cout << "Total time:" << std::endl;
			for (int j = 0; j < indent_num_; j++) {
				std::cout << "\t";
			}
			std::cout << std::fixed << "\033[33m" << total_time << "\033[0m" << ".sec" << std::endl;

			std::cout << "--------------------------------" << std::endl;
		}
	private:
		std::list<std::pair<std::string, double>> time_pairs_;

		std::chrono::system_clock::time_point start_time_;
		std::chrono::system_clock::time_point end_time_;

		std::list<double> time_duration_;

		int indent_num_ = 1;
	}; // class TimeCount
	
} // namespace PowerDiagramGenerator
#endif
