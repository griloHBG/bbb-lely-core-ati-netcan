set title "Motor Log"
set yrange [-50:50]
#set ytics (105, 100, 95, 90, 85, 80)
#set xrange [50:253]
#set lmargin 9
#set rmargin 2
plot 'qd0.000000_K0.010000_B0.010000.csv' using 0:5 notitle with lines
plot 'qd0.000000_K10.000000_B2.000000.csv' using 0:5 notitle with lines
