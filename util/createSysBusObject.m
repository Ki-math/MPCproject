function sysBus = createSysBusObject(numState,numMV,numMD,numOV)
% Create object
sysBus = Simulink.Bus;

% Create element
A = Simulink.BusElement;
A.Name = 'A';
A.Dimensions = [numState,numState];
A.DimensionsMode = 'Fixed';

B = Simulink.BusElement;
B.Name = 'B';
B.Dimensions = [numState,numMV];
B.DimensionsMode = 'Fixed';

if numMD == 0
    numMD = 1;      % Not allow 0, it must be grater than 0.
end
Bd = Simulink.BusElement;
Bd.Name = 'Bd';
Bd.Dimensions = [numState,numMD];
Bd.DimensionsMode = 'Fixed';

C = Simulink.BusElement;
C.Name = 'C';
C.Dimensions = [numOV,numState];
C.DimensionsMode = 'Fixed';

U = Simulink.BusElement;
U.Name = 'U';
U.Dimensions = numMV;
U.DimensionsMode = 'Fixed';

MD = Simulink.BusElement;
MD.Name = 'MD';
MD.Dimensions = numMD;
MD.DimensionsMode = 'Fixed';

Y = Simulink.BusElement;
Y.Name = 'Y';
Y.Dimensions = numOV;
Y.DimensionsMode = 'Fixed';

X = Simulink.BusElement;
X.Name = 'X';
X.Dimensions = numState;
X.DimensionsMode = 'Fixed';

f0 = Simulink.BusElement;
f0.Name = 'f0';
f0.Dimensions = numState;
f0.DimensionsMode = 'Fixed';

% Insert elements to the sysBus object
sysBus.Elements = [A B Bd C U MD Y X f0];

end