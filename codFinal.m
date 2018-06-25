%Este código consiste na calibração da webcam do computador utilizando um
%tabuleiro de xadrez.
%É feita a leitura das imagens e plotado um cubo, um dragão ou um tatu,
%definido pelo usuário, assim como a rotação e o tamanho.
%--------------------------------------------------------------------------
%Mudar caminho para encontrar as imagens de calibração na viarável path.
%--------------------------------------------------------------------------
%Definir variáveis val, rot e L.
%--------------------------------------------------------------------------
%Para finalizar o programa, feche a janela da figura e pressione ctrl+c na 
%janela de comando do MATLAB.
%--------------------------------------------------------------------------

clear all
clc

%% Calibração da câmera

path = 'C:\Users\Caique Sanches\Dropbox\visao\trabalho3\TrabalhoCubo\Dataset';
% Define images to process
imageFileNames = {strcat(path,'\Image1.png'),...
    strcat(path,'\Image2.png'),...
    strcat(path,'\Image3.png'),...
    strcat(path,'\Image4.png'),...
    strcat(path,'\Image5.png'),...
    strcat(path,'\Image6.png'),...
    strcat(path,'\Image7.png'),...
    strcat(path,'\Image8.png'),...
    strcat(path,'\Image9.png'),...
    strcat(path,'\Image10.png'),...
    strcat(path,'\Image11.png'),...
    strcat(path,'\Image12.png'),...
    strcat(path,'\Image13.png'),...
    strcat(path,'\Image14.png'),...
    strcat(path,'\Image15.png'),...
    strcat(path,'\Image16.png'),...
    strcat(path,'\Image17.png'),...
    strcat(path,'\Image18.png'),...
    strcat(path,'\Image19.png'),...
    strcat(path,'\Image20.png'),...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Generate world coordinates of the corners of the squares
squareSize = 18;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);


%% Inicializa Camera e parâmetros

cam = webcam();
figure
L=1; %Tamanho do cubo (L*18mm) e proporção da nuvem de pontos
rot=0; %Rotação do cubo ou figura
val=2; %Define Figura a ser mostrada
% val = 1 --> Cubo
% val = 2 --> Dragão
% val = 3 --> Tatu

meio = floor(length(worldPoints)/2); %ponto médio do tabuleiro
cxy=[worldPoints(meio,1),worldPoints(meio,2)]; % coordenada do ponto medio do tabuleiro

%% Calcula pontos da figura

switch val
    
    % Caso 1 -- Cubo
    case 1
        pontoshom = 0;
        pontosuv = 0;
        while (true)

            originalImage = rgb2gray(snapshot(cam)); %leitura da webcam
            undistortedImage = undistortImage(originalImage, cameraParams); %Retira Distorção
            idisp(undistortedImage)
            hold on;

            [imagePoints2,boardSize2,imagesUsed2] = detectCheckerboardPoints(undistortedImage); %Detecção tabuleiro na webcam

            if size(imagePoints2,1) == size(worldPoints,1) % verifica se o tabuleiro foi encontrado corretamente
    
                % Obtenção da matriz da camera
                [rotationMatrix, translationVector] = extrinsics(imagePoints2,worldPoints, cameraParams);
                P = cameraMatrix(cameraParams,rotationMatrix,translationVector);
                P = P'; 
                
                %Definição e transformação dos pontos do cubo
                pontoshom = P*[cxy(1);cxy(2);0;1];
                pontosuv = [pontoshom(1)/pontoshom(3) pontoshom(2)/pontoshom(3)];

                pontoshom2 = P*[cxy(1)+(cosd(rot)*(18*L));cxy(2)+(sind(rot)*(18*L));0;1];
                pontosuv2 = [pontoshom2(1)/pontoshom2(3) pontoshom2(2)/pontoshom2(3)];

                pontoshom3 = P*[cxy(1)+((cosd(rot)-sind(rot))*(18*L));cxy(2)+((cosd(rot)+sind(rot))*(18*L));0;1];
                pontosuv3 = [pontoshom3(1)/pontoshom3(3) pontoshom3(2)/pontoshom3(3)];

                pontoshom4 = P*[cxy(1)-(sind(rot)*(18*L));cxy(2)+(cosd(rot)*(18*L));0;1];
                pontosuv4 = [pontoshom4(1)/pontoshom4(3) pontoshom4(2)/pontoshom4(3)];

                pontoshom5 = P*[cxy(1);cxy(2);-(18*L);1];
                pontosuv5 = [pontoshom5(1)/pontoshom5(3) pontoshom5(2)/pontoshom5(3)];

                pontoshom6 = P*[cxy(1)+(cosd(rot)*(18*L));cxy(2)+(sind(rot)*(18*L));-(18*L);1];
                pontosuv6 = [pontoshom6(1)/pontoshom6(3) pontoshom6(2)/pontoshom6(3)];

                pontoshom7 = P*[cxy(1)+((cosd(rot)-sind(rot))*(18*L));cxy(2)+((cosd(rot)+sind(rot))*(18*L));-(18*L);1];
                pontosuv7 = [pontoshom7(1)/pontoshom7(3) pontoshom7(2)/pontoshom7(3)];

                pontoshom8 = P*[cxy(1)-(sind(rot)*(18*L));cxy(2)+(cosd(rot)*(18*L));-(18*L);1];
                pontosuv8 = [pontoshom8(1)/pontoshom8(3) pontoshom8(2)/pontoshom8(3)];

                %Definição dos pontos de cada face do cubo
                x = [pontosuv(1);pontosuv5(1);pontosuv6(1);pontosuv2(1)];
                y = [pontosuv(2);pontosuv5(2);pontosuv6(2);pontosuv2(2)];

                x2 = [pontosuv6(1);pontosuv2(1);pontosuv3(1);pontosuv7(1)];
                y2 = [pontosuv6(2);pontosuv2(2);pontosuv3(2);pontosuv7(2)];

                x3 = [pontosuv3(1);pontosuv7(1);pontosuv8(1);pontosuv4(1)];
                y3 = [pontosuv3(2);pontosuv7(2);pontosuv8(2);pontosuv4(2)];

                x4 = [pontosuv8(1);pontosuv4(1);pontosuv(1);pontosuv5(1)];
                y4 = [pontosuv8(2);pontosuv4(2);pontosuv(2);pontosuv5(2)];
        
                x5 = [pontosuv8(1);pontosuv7(1);pontosuv6(1);pontosuv5(1)];
                y5 = [pontosuv8(2);pontosuv7(2);pontosuv6(2);pontosuv5(2)];

                
                %Plot de cada face do cubo
                xy4=fill(x4,y4,'g')
                set(xy4,'facealpha',0.5)
                xy3=fill(x3,y3,'g')
                set(xy3,'facealpha',0.5)
                xy2=fill(x2,y2,'g')
                set(xy2,'facealpha',0.5)
                xy=fill(x,y,'g')
                set(xy,'facealpha',0.5)
                xy5=fill(x5,y5,'g')
                set(xy5,'facealpha',0.5)

            end

        end
        
    % Caso 2 -- Dragão
    case 2
        
        pontoshom = 0;
        pontosuv = 0;
        
        ptCloud = pcread('dragon.ply'); %Leitura do arquivo .ply
        
        %Transformação de Translação, Rotação e Tamanho
        A = [cosd(rot) 0 -sind(rot) 0;0 1 0 0;sind(rot) 0 cosd(rot) 0;0 -0.052 0 1];
        tform1 = affine3d(A);
        ptCloudTformed = pctransform(ptCloud,tform1);
        Dragon = (ptCloudTformed.Location)*500*L;
        
        
        
        while(true)
            
            originalImage = rgb2gray(snapshot(cam));%Leitura da webcam
            undistortedImage = undistortImage(originalImage, cameraParams); %Retira Distorção
            idisp(undistortedImage)
            hold on;

            [imagePoints2,boardSize2,imagesUsed2] = detectCheckerboardPoints(undistortedImage);%Detecção tabuleiro na webcam

            if size(imagePoints2,1) == size(worldPoints,1) % verifica se o tabuleiro foi encontrado corretamente
    
                %Obtenção da matriz da câmera 
                [rotationMatrix, translationVector] = extrinsics(imagePoints2,worldPoints, cameraParams);
                P = cameraMatrix(cameraParams,rotationMatrix,translationVector);
                P = P';
                
                %Transformação da nuvem de pontos para cordenadas
                %cartesianas da imagem
                pos=1;
                for i=1:length(Dragon)
                    if mod(i,75)==0
                        pontoshom = P*[-Dragon(i,1)+cxy(1);Dragon(i,3)+cxy(2);-Dragon(i,2);1];
                        pontosuv(pos,1) = pontoshom(1)/pontoshom(3);
                        pontosuv(pos,2) = pontoshom(2)/pontoshom(3);
                        pos = pos+1;
                    end
                end
                %Plot da nuvem de pontos
                plot(pontosuv(:,1),pontosuv(:,2),'.');
            
            end
        end
        
        
    case 3
        
        pontoshom = 0;
        pontosuv = 0;
        ptCloud = pcread('Armadillo.ply');
        A = [cosd(rot) 0 -sind(rot) 0;0 1 0 0;sind(rot) 0 cosd(rot) 0;0 54 0 1];
        tform1 = affine3d(A);
        ptCloudTformed = pctransform(ptCloud,tform1);
        Armadillo = ((ptCloudTformed.Location)*L)/2;
        
        
        
        while(true)
            
            originalImage = rgb2gray(snapshot(cam));%Leitura da webcam
            undistortedImage = undistortImage(originalImage, cameraParams);%Retira Distorção
            idisp(undistortedImage)
            hold on;

            [imagePoints2,boardSize2,imagesUsed2] = detectCheckerboardPoints(undistortedImage);%Detecção tabuleiro na webcam

            if size(imagePoints2,1) == size(worldPoints,1)% verifica se o tabuleiro foi encontrado corretamente
    
                %Obtenção da matriz da câmera
                [rotationMatrix, translationVector] = extrinsics(imagePoints2,worldPoints, cameraParams);
                P = cameraMatrix(cameraParams,rotationMatrix,translationVector);
                P = P';
                
                %Transformação da nuvem de pontos para cordenadas
                %cartesianas da imagem
                pos=1;
                for i=1:length(Armadillo)
                    if mod(i,90)==0
                        pontoshom = P*[Armadillo(i,1)+cxy(1);-Armadillo(i,3)+cxy(2);-Armadillo(i,2);1];
                        pontosuv(pos,1) = pontoshom(1)/pontoshom(3);
                        pontosuv(pos,2) = pontoshom(2)/pontoshom(3);
                        pos = pos+1;
                    end
                end
                %Plot da nuvem de pontos
                plot(pontosuv(:,1),pontosuv(:,2),'.');
            
            end
        end  
        
        
    otherwise
        
end


