clear all;
clc;

freq = 77e9;
max_range = 200;
range_resolution = 1;
max_velocity = 100;
c = 3e8;
lambda = freq/c ;

%% User Defined Range and Velocity of target
range = 110;
velocity = -20 ;
d_res = 1;  % range resolution [m]

bandwidth = c/(2*d_res);
%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.


%Operating carrier frequency of Radar 
range_max = 200;
                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

Tchirp = 5.5*2*(range_max)/c;
slope = bandwidth / Tchirp;

disp(Tchirp)
disp(slope)

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    r_t(i) = range + velocity*t(i);
    td(i) = (2*r_t(i))/c;
    
    Tx(i)   = cos(2*pi*(freq*t(i)+ (slope*t(i)^2)/2));
    Rx(i)   = cos(2*pi*(freq*(t(i)-td(i))+ (slope*(t(i)-td(i))^2)/2));  
end

Mix = Tx.*Rx;

disp(Mix)

%% RANGE MEASUREMENT

 % *%TODO* :
% reshape the vector into Nr*Nd array. Nr and Nd here would also define 
% the size of Range and Doppler FFT respectively.
Mix = reshape(Mix,[Nr,Nd]);

 % *%TODO* :
% run the FFT on the beat signal along the range bins dimension (Nr) 
% and normalize.
Y = fft(Mix,[],1); % Y = fft(Mix,Nr);
P = Y./Nr;

 % *%TODO* :
% Take the absolute value of FFT output
P2 = abs(P);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
P1 = P2(1:Nr/2+1);

 % *%TODO* : 
% plot FFT output 
figure ('Name','Range from First FFT');plot(P1) 
disp(P1)
%plotting the range
axis ([0 200 0 0.5]);

Mix = reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure('Name','2D FFT output - Range Doppler Map'),surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 7; % number of training cells for range
Td = 7; % number of training cells for doppler 
% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 2; % number of guard cells for range
Gd = 2; % number of guard cells for doppler
% *%TODO* :
% offset the threshold by SNR value in dB
offset  = 5; % offset the threshold by SNR value in [dB]

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
range = 2*(Tr+Gr)+1;
doppler = 2*(Td+Gd)+1;
noise_level = ones(doppler,range);

% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
for i=(1+Td):(1+Td+Gd*2)
    for j= (1+Tr):(1+Tr+Gr*2)
        noise_level(i,j) = 0;
    end
end
noise_level = noise_level/sum(noise_level,'all');
threshold_CFAR = pow2db(conv2(db2pow(RDM),noise_level,'same')) + offset;

RDM = double(RDM >= threshold_CFAR);

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
RDM(union(1:(Tr+Gr),end-(Tr+Gr-1):end),:) = 0;  % truncated map range
RDM(:,union(1:(Td+Gd),end-(Td+Gd-1):end)) = 0; 
% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
% figure,surf(doppler_axis,range_axis,'replace this with output');
% colorbar;

figure('Name','Output of the 2D CFAR process'),surf(doppler_axis,range_axis,RDM);
colorbar;
 
 