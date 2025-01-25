#ifndef TIME_COUNT_HPP_
#define TIME_COUNT_HPP_

#include <list>
#include <string>
#include <iostream>
#include <chrono>

namespace PowerDiagramGenerator {
	class TimeCount {
	public:
		TimeCount() = default;
		~TimeCount() {
			(std::list<std::pair<std::string, double>>()).swap(time_pairs_);
			(std::list<double>()).swap(time_duration_);
		}

		void start() {
			start_time_ = std::chrono::system_clock::now();
		}

		void stop() {
			const auto cur_time = std::chrono::system_clock::now();
			const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(cur_time - start_time_);
			const double duration_time = static_cast<double>(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;

			time_duration_.push_back(duration_time);
		}

		void reset() {
			(std::list<double>()).swap(time_duration_);
		}

		void count_time(const std::string& func_name) {
			if (time_duration_.empty()) {
				stop();
			}

			double total_time = 0;
			for (auto td_it = time_duration_.begin(); td_it != time_duration_.end(); ++td_it) {
				total_time += *td_it;
			}
			time_pairs_.emplace_back(func_name, total_time);

			reset();
		}

		void cover_count_time(const std::string& func_name) {
			if (time_duration_.empty()) {
				stop();
			}

			double total_time = 0;
			for (auto td_it = time_duration_.begin(); td_it != time_duration_.end(); ++td_it) {
				total_time += *td_it;
			}

			time_pairs_.pop_back();
			time_pairs_.emplace_back(func_name, total_time);

			reset();
		}

		void print_time_count() const {
			double total_time = 0;

			std::cout << "---------- TIME COUNT ----------" << std::endl;
			for (auto tp_it = time_pairs_.begin(); tp_it != time_pairs_.end(); ++tp_it) {
				std::cout << tp_it->first << ":" << std::endl;
				for (int j = 0; j < indent_num_; j++) {
					std::cout << "\t";
				}
				std::cout << std::fixed << "\033[33m" << tp_it->second << "\033[0m" << ".sec" << std::endl;

				total_time += tp_it->second;
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
	
}
#endif
