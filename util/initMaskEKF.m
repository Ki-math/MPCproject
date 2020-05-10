function p = initMaskEKF(stateFunction,measurementFunction,jacobianStateFunction,jacobianMeasurmentFunction)

p.stateFunction = stateFunction;
p.measurementFunction = measurementFunction;
p.jacobianStateFunction = jacobianStateFunction;
p.jacobianOfMeasurementFunction = jacobianMeasurmentFunction;

end