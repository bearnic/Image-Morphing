%Nicholas Tylek
%ECES 436
%Assignment 4
%Image Morphing
%5/31/2016

clear all;
close all
clc;

%Note that images are of the same size 400*300
image1 = imread('face.jpg');
imshow(image1);
image2 = imread('ben.jpg');
figure;
imshow(image2);

%NOTE Delaney triangulation can be used to find common shape of faces to
%combine. 

%extract r,g,b matrices from each image
r1 = image1(:,:,1);
g1 = image1(:,:,2);
b1 = image1(:,:,3);

r2 = image2(:,:,1);
g2 = image2(:,:,2);
b2 = image2(:,:,3);

%find the average r,g,b matrices of both
rAvg = .5.*r1+.5.*r2;
gAvg = .5.*g1+.5.*g2;
bAvg = .5.*b1+.5.*b2;
crossAvg = cat(3,rAvg,gAvg,bAvg);
figure;
imshow(crossAvg);

if ~exist('points.mat', 'file')
    [row1, column1, ~] = size(image1);
    [row2, column2, ~] = size(image2);
    row            = max(row1, row2); 
    column            = max(column1, column2);
    image1Pad       = padarray(image1, [row-row1, column-column1], 'replicate', 'post');
    image2Pad       = padarray(image2, [row-row2, column-column2], 'replicate', 'post');
  
    [image2Points, image1Points] = cpselect(image2Pad, image1Pad,'Wait', true);

    image1Points = [image1Points; 0, 0; 0, row; column, row; column, 0];
    image2Points = [image2Points; 0, 0; 0, row; column, row; column, 0];
    imagePoints = ( image1Points + image2Points ) / 2;
    save('points.mat', 'image1Points', 'image2Points', 'imagePoints')
end

if ~exist('image1Points', 'var') || ~exist('image2Points', 'var')
    load('points.mat');
    [row1, column1, ~] = size(image1);
    [row2, column2, ~] = size(image2);
    row            = max(row1, row2); 
    column            = max(column1, column2);
    image1Pad       = padarray(image1, [row-row1, column-column1], 'replicate', 'post');
    image2Pad       = padarray(image2, [row-row2, column-column2], 'replicate', 'post');
end

imagePoints = ( image1Points + image2Points ) / 2;
DT = delaunayTriangulation(imagePoints);
figure;
triplot(DT)

for n = 0:0.05:1

    a=1-n;
    b=0+n;
    
    %create crossfade animation with 20 frames
    rCom =a.*r1+b.*r2;
    gCom =a.*g1+b.*g2;
    bCom = a.*b1+b.*b2;
    CrossCom = cat(3,rCom,gCom,bCom);
    
    [imind,cm] = rgb2ind(CrossCom,256);
    if n == 0;
        imwrite(imind,cm,'crossfade.gif','gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,'crossfade.gif','gif','WriteMode','append');
    end
    
    %Get the intermediate critical points
    WarpPoints = a * image1Points + b * image2Points;
    
    array  = [repmat(1:column,1,row);reshape(repmat(1:row,column,1),...
        [1 row*column]);ones(1,row*column)];
    indexDelauney = tsearchn([WarpPoints(:,1) WarpPoints(:,2)], DT, [array(1,:)' array(2,:)']);
    
    for i = 1 : size(DT,1)
        warpTransform = [[WarpPoints(DT(i, 1), :); WarpPoints(DT(i, 2), :); ...
            WarpPoints(DT(i, 3),:)]'; ones(1,3)];
        ImageTransform1 = [[image1Points(DT(i, 1), :); image1Points(DT(i, 2), :); ...
            image1Points(DT(i, 3), :)]'; ones(1,3)] / warpTransform;
        ImageTransform2 = [[image2Points(DT(i, 1), :); image2Points(DT(i, 2), :);...
            image2Points(DT(i, 3), :)]'; ones(1,3)] / warpTransform;
        pixel1 = ImageTransform1 * array(:,indexDelauney == i);
        p1in = round(bsxfun(@rdivide, pixel1,pixel1(end, :)));
        p1(:,indexDelauney == i) = bsxfun(@min,bsxfun(@max,p1in,[1 1 1]'),[column,row,1]');
        
        pixel2 = ImageTransform2 * array(:,indexDelauney == i);
        p2in = round(bsxfun(@rdivide, pixel2,pixel2(end, :)));
        p2(:,indexDelauney == i) = bsxfun(@min,bsxfun(@max,p2in,[1 1 1]'),[column,row,1]');
    end

    for i = 1 : row*column
        if isnan(indexDelauney(i))
            continue;
        end
        image1Warped(array(2,i),array(1,i), :) = image1Pad(p1(2, i), p1(1, i), :);
        image2Warped(array(2,i),array(1,i), :) = image2Pad(p2(2, i), p2(1, i), :);
    end
    
    FinalMorphFrame = a*image1Warped + b*image2Warped;
    [imind,cm] = rgb2ind(FinalMorphFrame,256);
    if n == 0;
        imwrite(imind,cm,'morph.gif','gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,'morph.gif','gif','WriteMode','append');
    end
    
end
DISP('finished')