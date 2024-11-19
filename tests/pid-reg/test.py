import pandas
from sklearn import linear_model

df = pandas.read_csv("data.csv")

X = df[['kP', 'kI', 'kD']]
y = df['error']

regr = linear_model.LinearRegression()
regr.fit(X, y)

predictedCO2 = regr.predict([[3300, 1300]])

print(predictedCO2)