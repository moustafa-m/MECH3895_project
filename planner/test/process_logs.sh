#!/bin/bash

RED='\033[0;31m'
NC='\033[0m'

Usage()
{
    printf "Usage: $0 [-h] -d\n"
    printf "Options:\n"
    printf "h:\n\t Prints this message\n"
    printf "d:\n\t Directory containing .log files that will be processed\n"
}

if [ -z "$*" ]; then
    Usage
    exit 0
fi

dir=""
overall_log="overall_avg.log"
merged_log="merged.log"
processed_log="processed.log"

while getopts ":hd:" opt; do
    case "${opt}" in
        h)
            Usage
            exit 0
            ;;
        d)
            dir=$OPTARG
            ;;
        :)
            printf "${RED}Missing -d argument!${NC}\n"
            exit 0
            ;;
        \?)
            printf "${RED}Unrecognised option -${OPTARG}!${NC}\n"
            Usage
            exit 0
            ;;
        *)
            Usage
            exit 0
            ;;
    esac
done

if [ -z  "$dir" ]; then
    printf "${RED}Please input a valid directory!${NC}\n"
    Usage
    exit 1
fi

cd "$dir"

# ensure files are empty to avoid duplication when directory is processed again
> "$processed_log"
> "$merged_log"
> "$overall_log"

if [ $? -ne 0 ]; then
    printf "${RED}Error encountered when entering directory!${NC}\n"
    exit 1
fi

for filename in *.log; do
    if [[ "$filename" == "$processed_log" || "$filename" == "$overall_log" || "$filename" == "$merged_log" ]]; then
        continue
    fi

    printf "Processing $filename...\n"
    file=$dir$filename
    # printf "${file}\n"
    rosrun planner data_processor "$file"
done

find . -maxdepth 1 -iname '*.log' ! -name "$merged_log" ! -name "${overall_log}" ! -name "$processed_log" -exec cat {} +> "$merged_log"

dir+="merged.log"
printf "Processing merged.log...\n"
rosrun planner data_processor "$dir" "$overall_log"
