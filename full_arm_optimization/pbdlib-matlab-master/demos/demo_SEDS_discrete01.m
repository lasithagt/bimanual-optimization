function demo_SEDS_discrete01
% Discrete autonomous dynamical system with state-space encoding using GMM, with GMR 
% used for reproduction by using a constrained optimization similar to the SEDS approach.
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
% 
% Copyright (c) 2016 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 6; %Number of states in the GMM
model.nbVar = 4; %Number of variables [x1,x2,dx1,dx2]
nbData = 100; %Length of each trajectory
nbSamples = 5; %Number of demonstrations


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/S.mat');
Data=[];
for n=1:nbSamples
	s(n).Data0 = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData+1)); %Resampling
	s(n).Data0 = s(n).Data0 - repmat(s(n).Data0(:,end),1,nbData+1); %Center at 0
	Data0(:,n) = s(n).Data0(:,1);
	s(n).Data = [s(n).Data0(:,1:end-1); s(n).Data0(:,2:end)]; %Velocity computation	
	Data = [Data, s(n).Data]; 
end
Data0 = mean(Data0,2);


%% Learning 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_kmeans(Data, model);
model = init_GMM_kbins(Data, model, nbSamples);
model = EM_GMM(Data, model);


%% Constrained optimization 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Refinement through constrained optimization...');
in=[1:2]; out=[3:4]; 
for i=1:model.nbStates
  H(:,i) = model.Priors(i) * gaussPDF(Data(in,:),model.Mu(in,i),model.Sigma(in,in,i)); %Weight based on GMM
end
H = H ./ repmat(sum(H,2),1,model.nbStates);    

options = optimset('Algorithm','active-set','display','notify'); %'large-scale' could be used instead 'active-set'
X=Data(in,:); Y=Data(out,:);
for i=1:model.nbStates
  %Initialization through weighted least-squares regression (dx=A*x)
  A0 = [(X * diag(H(:,i).^2) * X') \ X * diag(H(:,i).^2) * Y']';
	%Refined solution through constrained optimization
  model.A(:,:,i) = fmincon(@(A)myfun(A,X,Y,diag(H(:,i))),A0,[],[],[],[],[],[],@(A)mycon(A),options);
end


%% Reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Instable GMR reproduction
rData0 = zeros(2,nbData);
rData0(:,1) = Data0;
for t=2:nbData
	rData0(:,t) = GMR(model, rData0(:,t-1), [1:2], [3:4]); 
end

%Stable SEDS reproduction
rData = zeros(2,nbData);
rData(:,1) = Data0;
for t=2:nbData
  for i=1:model.nbStates
    h(i,1) = model.Priors(i) * gaussPDF(rData(:,t-1), model.Mu(1:2,i), model.Sigma(1:2,1:2,i)); %Weight based on GMM
  end
  h = h / sum(h);
  %Compute position
  for i=1:model.nbStates
    rData(:,t) = rData(:,t) + h(i) * model.A(:,:,i) * rData(:,t-1); 
	end
end


%% Plot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 20 24],'position',[10,10,1300,700],'color',[1 1 1]); hold on; axis off; 
for i=1:nbSamples
	plot(Data(1,(i-1)*nbData+1:i*nbData), Data(2,(i-1)*nbData+1:i*nbData), '-','lineWidth',1,'color',[.4 .4 .4]);
end
for i=1:model.nbStates
	plotGMM(model.Mu(1:2,i), model.Sigma(1:2,1:2,i), [.6 .6 .6], .8);
end
hl(1)=plot(rData0(1,:), rData0(2,:), '-','lineWidth',2,'color',[1 0 0]);
hl(2)=plot(rData(1,:), rData(2,:), '-','lineWidth',2,'color',[0 .8 0]);
legend(hl,'Instable GMR reproduction','Stable SEDS reproduction');
axis equal;

%print('-dpng','graphs/demo_SEDS_discrete01.png');
pause;
close all;


%% Optimization function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function f = myfun(A,X,Y,W)
fTmp = (A*X-Y)*W;
f = norm(reshape(fTmp,size(Y,1)*size(Y,2),1));


%% Constraint function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [c,ceq] = mycon(A)
Atmp = (A+A')*.5;
[~,D] = eig(Atmp);
for j=1:size(A,1)
  polesR(j) = norm(D(j,j)) - .99; %margin on unit circle (discrete case)
end
%Force the poles to be in the unit circle
c = [polesR]; 
ceq = 0;
