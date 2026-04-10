data5 = readtable("J5 Angle.xlsx");
scatter(data5.Reported,data5.Error)
scatter(data5.Reported2,data5.Error2)

data3 = readtable("J3 Angle.xlsx");
scatter(data3.ReportedPosition,data3.Error)
data2 = readtable("J2 Angle.xlsx");
scatter(data2.ReportedPosition,data2.Error)


fitlm(data3.ReportedPosition,data3.Error)