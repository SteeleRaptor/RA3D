data = readmatrix("Pitch.xlsx");
x = data(:,1);
pitch = data(:,2);
plot(x,pitch);
model = fitlm(x,pitch);

x_predict = -50:.1:50;
y_predict = model.predict(x_predict');
plot(x_predict,y_predict)