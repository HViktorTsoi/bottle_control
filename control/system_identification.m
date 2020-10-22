data=csvread('/tmp/chassis_test.csv');
w_output=data(:,2);
w_input=data(:,4);
plot(w_input);hold;
plot(w_output);