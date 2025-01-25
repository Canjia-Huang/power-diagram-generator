#ifndef OPTIONS_HPP_
#define OPTIONS_HPP_

#ifndef OUTPUT_PATH
#define OUTPUT_PATH
#endif

#include <map>
#include <string>
#include <cstring>

namespace PowerDiagramGenerator {
	class Options {
	public:
		bool showHelp;
		bool exact;
		bool voronoi;
		bool bounding;
		bool timecount;
		double r;
		std::string output_path;
		std::vector<std::string> outputList; // store [OUTPUT_TYPE]
		std::vector<std::string> visualList; // store [VISUAL_TYPE]
	public:
		Options()
		: showHelp(false)
		, exact(false)
		, voronoi(false)
		, bounding(false)
		, timecount(false)
		, r(0)
		, output_path(OUTPUT_PATH)
		{
			boolArgsMap = {
				{"--bounding", &bounding}, {"--box", &bounding}, {"--b", &bounding},
				{"--exact", &exact}, {"--e", &exact},
				{"--help", &showHelp}, {"--h", &showHelp},
				{"--timecount", &timecount}, {"--tc", &timecount}, {"--t", &timecount},
				{"--voronoi", &voronoi}, {"--vor", &voronoi}, {"--v", &voronoi}
			};
			intArgsMap = {
			};
			doubleArgsMap = {
				{"-radius", &r}, {"-r", &r}
			};
			stringArgsMap = {
				{"-path", &output_path}, {"-op", &output_path}, {"-p", &output_path},
			};
		}

		void showOptions(std::ostream& o) const {
			o << "\t OPTIONS:\n";
			o << "\t --bounding|--box|--b: initialize the initial box for each point as the bounding box of the entire point cloud\n";
			o << "\t --exact|--e: completely exactly compute the power diagram, it may result in an increase in computational overhead, only for verification\n";
			o << "\t --help|--h: show this help\n";
			o << "\t -output|-o=[OUTPUT_TYPE]: generate the output of the power diagram in the form of the specified data type\n";
			o << "\t\t OUTPUT_TYPE:\n";
			o << "\t\t neighbor|n: the neighbors information between points\n";
			o << "\t -path|-op|-p=[string]: set the output path [default ..//..//data//]\n";
			o << "\t -radius|-r=[double]: set the radius of the initial box for each point [default 0]\n";
			o << "\t --timecount|--tc|--t: Output the time count information\n";
			o << "\t --voronoi|--vor|--v: compute the voronoi diagram\n";
			o << "\t -visual|-vis|-v=[VISUAL_TYPE]: generate the visual output of the power diagram in the form of the specified data type.obj\n";
			o << "\t\t VISUAL_TYPE:\n";
			o << "\t\t connection|c: the connection relationships between points\n";
			o << "\t\t polyhedron|p: the polyhedron of power cells\n";
			o << "\t\t wireframe|wire|w: the wireframe of power cells\n";
			o << "\t\n";
		}

		/** \brief Parse an input option.
		* \param[in, out] i: parse the argv[i]
		*					for options with input parameters, adjust i so that i+1 become the next option's index that should be parsed
		* \param[in] argc: the size of argv
		* \param[in] argv: the array of options
		* \retval whether this option is parsed successfully */
		bool parseAnOption(int& i, int argc, char const* const* argv) {
			if (strlen(argv[i]) == 0) return true;

			std::string option(argv[i]);
			std::string arg;

			// process [bool] type option
			auto boolIt = boolArgsMap.find(option);
			if (boolIt != boolArgsMap.end()) {
				*boolIt->second = true;
				return true;
			}

			// process [int] and [double] type option
			std::size_t found = option.find('=');
			if (found != std::string::npos) {
				arg = option.substr(found + 1);
				option = option.substr(0, found);
			}

			auto intIt = intArgsMap.find(option);
			if (intIt != intArgsMap.end()) {
				if (arg.empty() && i + 1 >= argc) return false;
				*intIt->second = std::atoi(arg.empty() ? argv[++i] : arg.c_str());
				return true;
			}

			auto doubleIt = doubleArgsMap.find(option);
			if (doubleIt != doubleArgsMap.end()) {
				if (arg.empty() && i + 1 >= argc) return false;
				*doubleIt->second = std::stod(arg.empty() ? argv[++i] : arg.c_str());

				/*if (i + 1 < argc) {
					std::string next_argv = std::string(argv[++i]);
					const size_t found_dot = next_argv.find('.');
					if (found_dot != std::string::npos) {
						const std::string arg2 = next_argv.substr(found_dot);
						*doubleIt->second += std::stod(arg2);
					}
				}*/
				return true;
			}

			// process [string] type option
			auto stringIt = stringArgsMap.find(option);
			if (stringIt != stringArgsMap.end()) {
				if (arg.empty() && i + 1 >= argc) return false;
				*stringIt->second = arg.empty() ? argv[++i] : arg.c_str();
				return true;
			}

			// process optional types option
			if ((option == "-output" || option == "-o") && (!arg.empty() || i + 1 < argc)) {
				outputList.emplace_back(arg.empty() ? argv[++i] : arg.c_str());
				return true;
			}
			if ((option == "-visual" || option == "-vis" || option == "-v") && (!arg.empty() || i + 1 < argc)) {
				visualList.emplace_back(arg.empty() ? argv[++i] : arg.c_str());
				return true;
			}

			return false;
		}

		/** \brief Parse input options.
		* \param[in] argc: the size of argv
		* \param[in] argv: the array of options
		* \retval argc */
		int parseOptions(int argc, char const* const* argv) {
			for (int i = 1; i < argc; ++i) {
				// std::cout << "prase:" << " " << argv[i] << std::endl;
				if (parseAnOption(i, argc, argv) == false) return i;
			}
			return argc;
		}
	private:
		std::map<std::string, bool*> boolArgsMap;
		std::map<std::string, int*> intArgsMap;
		std::map<std::string, double*> doubleArgsMap;
		std::map<std::string, std::string*> stringArgsMap;
	};

	Options& getOptions() {
		static Options s_options;
		return s_options;
	}
} // namespace PowerDiagramGenerator

#endif
