#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <iomanip>
#include <ros/package.h>
#include <boost/filesystem/operations.hpp>

class DataProcessor
{
public:
    DataProcessor(int argc, char** argv)
    {
        file_ = ros::package::getPath("planner") + "/test/" + argv[1];
        
        if (!boost::filesystem::exists(file_))
        {
            std::cerr << "Input file does not exist!\n";
            exit(-1);
        }

        if (argc > 3) { output_path_ = argv[3]; }
        else { output_path_ = boost::filesystem::path(file_).parent_path().string(); }

        if (!boost::filesystem::exists(output_path_))
        {
            std::cerr << "Output path does not exist!\n";
            exit(-1);
        }

        if (argc < 3)
        {
            processed_file_ = output_path_ + "/processed.log";
        }
        else
        {
            processed_file_ = argv[2];
            processed_file_ = processed_file_.substr(0, processed_file_.find_last_of("."));
            processed_file_ = output_path_ + "/" + processed_file_ + ".log";
        }
    }

    ~DataProcessor()
    {

    }

    void run()
    {
        std::ifstream log_file;
        log_file.open(file_);

        std::fstream processed_file;
        processed_file.open(processed_file_, std::fstream::app);

        double total_plan_time = 0, total_execution_time = 0, total_free_area = 0, total_area = 0;
        int total_num_success = 0, total_num_partial = 0, total_num_grasp = 0, total_num_actions = 0, total_num_objs = 0;

        FileData data;
        std::string line;
        int num_lines = 0;

        while (std::getline(log_file, line))
        {
            num_lines++;

            std::string str;
            std::stringstream ss;
            std::istringstream iss(line);
            while (std::getline(iss, str, ',')) { ss << str << " "; }

            double plan_time, execution_time, free_area, surface_area;
            int solution_found, partial_solution, grasp_success, num_actions, num_objs;

            ss >> solution_found >> partial_solution >> grasp_success >> num_actions
                >> plan_time >> execution_time >> num_objs >> free_area >> surface_area;

            total_num_success += solution_found;
            total_num_partial += partial_solution;
            total_num_grasp += grasp_success;
            total_num_actions += num_actions;
            total_plan_time += plan_time;
            total_execution_time += execution_time;
            total_num_objs += num_objs;
            total_free_area += free_area;
            total_area += surface_area;
        }
        
        data.success_rate = double(total_num_success)/num_lines;
        data.partial_rate = double(total_num_partial)/num_lines;
        data.grasp_rate = double(total_num_grasp)/num_lines;
        data.avg_num_actions = double(total_num_actions)/num_lines;
        data.avg_plan_time = total_plan_time/num_lines;
        data.avg_execution_time = total_execution_time/num_lines;
        data.avg_num_objects = double(total_num_objs)/num_lines;
        data.avg_free_space = total_free_area/num_lines;
        data.avg_surface_area = total_area/num_lines;
        
        processed_file << std::setprecision(3) << data << "\n";

        std::cout << "Done! Read " << num_lines << " lines\nData has been saved to:\n"
            << processed_file_ << "\n\n";
    }

private:
    struct FileData
    {
        double success_rate = 0;
        double partial_rate = 0;
        double grasp_rate = 0;
        double avg_num_actions = 0;
        double avg_plan_time = 0;
        double avg_execution_time = 0;
        double avg_num_objects = 0;
        double avg_free_space = 0;
        double avg_surface_area = 0;

        friend std::ostream& operator<<(std::ostream& os, const FileData& data)
        {
            os << data.success_rate << "," << data.partial_rate << "," << data.grasp_rate
                << "," << data.avg_num_actions << "," << data.avg_plan_time << ","
                << data.avg_execution_time << "," << data.avg_num_objects << ","
                << data.avg_free_space << "," << data.avg_surface_area;
            return os;
        }
    };

    std::string file_;
    std::string output_path_;
    std::string processed_file_;

};

void usage()
{
    std::cout << "data_processor.cpp: program to process logs into a single csv file of averages.\n\n"
        << "Usage:\n\n"
        << "    rosrun planner data_processor Arg1 [ Arg2 ] [ Arg3 ]\n\n"
        << "Output files:\n\n"
        << "    processed.log - averages of input log file. Located in same directory as original file by default. Name can be changed via Arg2.\n\n"
        << "Inputs:\n\n"
        << "    Arg 1: path to file to process, MUST be inside /logs directory.\n\n"
        << "    Arg 2: output file name. (extension part is ignored)\n\n"
        << "    Arg 3: output directory, can be anywhere in system. It will NOT be created if it doesn't exist.\n\n"
        << "Example:\n\n"
        << "    rosrun planner data_processor logs/cabinet/file.log\n";
}

int main(int argc, char** argv)
{
    if (argc == 1)
    {
        usage();
        return EXIT_SUCCESS;
    }

    DataProcessor processor(argc, argv);
    processor.run();
    
    return EXIT_SUCCESS;
}
