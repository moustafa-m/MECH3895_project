#!/bin/bash

RED='\033[0;31m'
NC='\033[0m'

Usage()
{
    printf "Usage: $0 [-h] -f [-a]\n"
    printf "Options:\n"
    printf "h:\n\t Prints this message.\n"
    printf "f:\n\t File to plot.\n"
    printf "a:\n\t Set this option if plotting averages using point-to-point lines.\n"
}

if [ -z "$*" ]; then
    Usage
    exit 0
fi

plot_avg=0
title_prefix=""
file=""

while getopts ":hf:a" opt; do
    case "${opt}" in
        h)
            Usage
            exit 0
            ;;
        f)
            file=$OPTARG
            ;;
        a)
            plot_avg=1
            title_prefix="Avg "
            printf "Plotting averages!\n"
            ;;
        :)
            printf "${RED}Missing -f argument!${NC}\n"
            Usage
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

if [ -z  "$file" ]; then
    printf "${RED}No filename input, -f option is required!${NC}\n"
    Usage
    exit 1
fi

echo > fit.log
echo "set datafile separator \",\"
plot_avg = ${plot_avg}

set term qt 0
set grid xtics ytics
set title \"${title_prefix}Planning Time against Num Objs\"
set xlabel \"Number of Objects\"
set ylabel \"Planning Time (s)\"

if (plot_avg > 0) {
    plot \"${file}\" using 7:(\$5+\$6) with linespoints title('data')
} else {
    f(x) = a*x + b
    fit f(x) '${file}' using 7:(\$5+\$6) via a, b
    plot \"${file}\" using 7:(\$5+\$6) with points title('data'), f(x) title('best fit')
}

set term qt 1
set title \"${title_prefix}Planning Time against Free Area Percentage\"
set xlabel \"Free Area (%)\"
set ylabel \"Plan Time (s)\"

if (plot_avg > 0) {
    plot \"${file}\" using (\$8*100/\$9):(\$5+\$6) with linespoints title('data')
} else {
    f(x) = a*x + b
    fit f(x) '${file}' using (\$8*100/\$9):(\$5+\$6) via a, b
    plot \"${file}\" using (\$8*100/\$9):(\$5+\$6) with points title('data'), f(x) title('best fit')
}

set term qt 2
set title \"${title_prefix}Number of Actions against Free Area Percentage\"
set xlabel \"Free Area (%)\"
set ylabel \"Number of Actions\"

if (plot_avg > 0) {
    plot \"${file}\" using (\$8*100/\$9):4 with linespoints title('data')
} else {
    f(x) = a*x + b
    fit f(x) '${file}' using (\$8*100/\$9):4 via a, b
    plot \"${file}\" using (\$8*100/\$9):4 with points title('data'), f(x) title('best fit')
}

set term qt 3
set title \"${title_prefix}Grasp Success Rate against Free Area Percentage\"
set xlabel \"Free Area (%)\"
set ylabel \"Grasp Success Rate\"

if (plot_avg > 0) {
    plot \"${file}\" using (\$8*100/\$9):3 with linespoints title('data')
} else {
    f(x) = a*x + b
    fit f(x) '${file}' using (\$8*100/\$9):3 via a, b
    plot \"${file}\" using (\$8*100/\$9):3 with points title('data'), f(x) title('best fit')
}

set term qt 4
set title \"${title_prefix}Grasp Success Rate against Num Objs\"
set xlabel \"Number of Objects\"
set ylabel \"Grasp Success Rate\"

if (plot_avg > 0) {
    plot \"${file}\" using 7:3 with linespoints title('data')
} else {
    f(x) = a*x + b
    fit f(x) '${file}' using 7:3 via a, b
    plot \"${file}\" using 7:3 with points title('data'), f(x) title('best fit')
}

pause -1 \"Press Enter to exit\"" > gnuplot.in

gnuplot gnuplot.in

if [ $? -eq 0 ]; then
    rm gnuplot.in
    rm fit.log
else
    printf "${RED}gnuplot encountered an error!${NC}\n"
fi
