
%
% Run Simulated annealing
k = [0.9 0.99 0.999]
for kTfac = k;

for n = 1:3;
nVehicles = 5;
nTasks = 18;
kTinit = 100;
kTend = 0.001;
%kTfac = 0.9;             
fprintf('nTasks = %.2f \n',nTasks);

deltaMat = load("/home/jendav/Videos/potts_spin_nn/Matrices/inputs/M=5_N=18/deltaMat.txt");
tVec = load("/home/jendav/Videos/potts_spin_nn/Matrices/inputs/M=5_N=18/tVec.txt");

nDimActive = nVehicles + nTasks; % Used for the kT change...

nDim = 2*nVehicles + nTasks;
vMat = zeros(nDim);
tic;
nNorm = 30; % Times to loop for normalizing V
normPrec = 0.01; % Precision for row and column sums
%vMat = load("/home/jendav/Videos/potts_spin_nn/Matrices/inputs/M=8_N=75/Vmat.txt");

% Create initial matrix (random)
doneCol = [];
for iRow = 1:nVehicles+nTasks
    if(iRow <= nVehicles)
        availCols = (nVehicles+1:nVehicles+nTasks);
    else
        availCols = (nVehicles+1:nDim);
    end
    % Remove diagonal
    availCols = setdiff(availCols,iRow);
    if(~isempty(doneCol))
        doCols = setdiff(availCols,doneCol);
    else
        doCols = availCols;
    end
    if(isempty(doCols))
        % This can only happen if the only available element in the last row 
        % is the diagonal. If so, then set the diagonal element to one and 
        % switch with another row
        vMat(iRow,iRow) = 1;
        tmpIndx = nVehicles + randi(nTasks-1,1);
        tmpRow = vMat(tmpIndx,:);
        vMat(tmpIndx,:) = vMat(iRow,:);
        vMat(iRow,:) = tmpRow;
    else
        nCols = length(doCols);
        tmpIndx = randperm(nCols);
        vMat(iRow,doCols(tmpIndx(1))) = 1;
        doneCol = [doneCol doCols(tmpIndx(1))];
    end
end

% Save V matrix
vMatOld = vMat;
costValueOld = 10^10;

vMatBest = NaN*ones(nDim);
costValueBest = 10^10;

kT = kTinit;

iIter = 0;
nLoops = 0;
nValid = 0;
tScale = NaN;
while kT > kTend
    iIter = iIter + 1;
    
    if(iIter > 1)
        %%% --- Start random step -------
        rowSwitch = randi(2); % random choice 50/50 between updating row and updating column
        if(rowSwitch == 1) % Do row
            rowIndx = randi(nVehicles+nTasks);
            indxA = find(vMat(rowIndx,:) == 1); % The old position
            if(rowIndx <= nVehicles)

                availCols = (nVehicles+1:nVehicles+nTasks);
                availCols = setdiff(availCols,indxA); % Diagonal not available here
                tmpIndx = randperm(length(availCols));
                indxB = availCols(tmpIndx(1)); % The new position
            else
                availCols = (nVehicles+1:nDim);
                availCols = setdiff(availCols,[indxA rowIndx]);
                tmpIndx = randperm(length(availCols));
                indxB = availCols(tmpIndx(1)); % The new position
            end

            rowIndxSwitch = NaN;
            for iRow = 1:nVehicles+nTasks
                if(vMat(iRow,indxB) == 1)
                    rowIndxSwitch = iRow;
                    break
                end
            end

            tmpRow = vMat(rowIndx,:);
            vMat(rowIndx,:) = vMat(rowIndxSwitch,:);
            vMat(rowIndxSwitch,:) = tmpRow;

        else % Do column
            colIndx = nVehicles + randi(nVehicles+nTasks);
            indxA = find(vMat(:,colIndx) == 1); % The old position
            if(colIndx > nVehicles+nTasks)
                availRows = (nVehicles+1:nVehicles+nTasks);

                availRows = setdiff(availRows,indxA); % Diagonal not available here
                tmpIndx = randperm(length(availRows));
                indxB = availRows(tmpIndx(1)); % The new position
            else
                availRows = (1:nVehicles+nTasks);
                availRows = setdiff(availRows,[indxA colIndx]);
                tmpIndx = randperm(length(availRows));
                indxB = availRows(tmpIndx(1)); % The new position
            end

            colIndxSwitch = NaN;
            for jCol = nVehicles+1:nDim
                if(vMat(indxB,jCol) == 1)

                    colIndxSwitch = jCol;
                    break
                end
            end

            tmpCol = vMat(:,colIndx);
            vMat(:,colIndx) = vMat(:,colIndxSwitch);
            vMat(:,colIndxSwitch) = tmpCol;
        end
        %%% --- End random step -------
    end
    
    % Check if we have a loop matrix 
    condNum = condest(eye(nDim) - vMat);
    if(isinf(condNum) | condNum > 100000)

        % Loop matrix - just accept and take a new random step but don't
        % update vMatOld
        costValue = NaN;
        nLoops = nLoops + 1;
    else
        % Compute cost
        
        % Compute exact propagator
        pMat = inv(eye(nDim) - vMat);

        % Compute left vector (L)
        tmpMat = tVec + diag(vMat'*deltaMat);
        lVec = pMat'*tmpMat;
        % compute right vector (R)
        tmpMat = tVec + diag(vMat*deltaMat');
        rVec = pMat*tmpMat;

        % Find largest values
        lIndx = zeros(nDim,1);
        lIndx(nDim-nVehicles+1:nDim) = 1;
        rIndx = zeros(nDim,1);
        rIndx(1:nVehicles) = 1;
        [lMaxValue,lMaxIndx] = max(lVec.*lIndx);
        [rMaxValue,rMaxIndx] = max(rVec.*rIndx);
        costValue = 0.5*(lMaxValue + rMaxValue);
        
        if(isnan(tScale)) % Just set first value encountered
            tScale = costValue;
        end
        % Save best encountered
        if(costValue < costValueBest)
            vMatBest = vMat;
            costValueBest = costValue;

        end

        nValid = nValid + 1;
    end
    
    if(~isnan(costValue)) % Only do this for valid solutions
        
        % Start Accept/reject step - correct simulated annealing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        accPfun = exp((costValueOld - costValue)/(tScale*kT));
        if(rand(1) < accPfun) % Accept, continue
            % Save values, to be able to revert
            vMatOld = vMat; 
            costValueOld = costValue;
        else % Do not accept, revert
            vMat = vMatOld;
            costValue = costValueOld;
        end
        % End Accept/reject step (correct simulated annealing) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end
    
    
    
    
    if(mod(iIter,nDimActive) == 0)
    % Only decrease temperature after having updated the matrix once
        kT = kT*kTfac;
    end
%    kT = kT*kTfac;


end

%fprintf('kT = %.2f\n',kT);



if(isnan(vMatBest))
    vMatSolution = vMat;
else
    vMatSolution = vMatBest;
end
%%
exTime = toc;

vecLetters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz';
taskLetters = 'abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz';


nDim = size(vMatSolution,1);
cTime = zeros(1,nVehicles);
for iV = 1:nVehicles

    indx = find(vMatSolution(iV,:) == 1);
    if(iV == 1)
        solStrA = ['S' vecLetters(iV) ' -> ' taskLetters(indx-nVehicles)];
        solStrB = ['max(' num2str(tVec(iV)) ' + ' num2str(deltaMat(iV,indx))  ' + ' num2str(tVec(indx))];  
    else
        solStrA = [solStrA ' & S' vecLetters(iV) ' -> ' taskLetters(indx-nVehicles)];
        solStrB = [solStrB ', ' num2str(tVec(iV)) ' + ' num2str(deltaMat(iV,indx)) ' + ' num2str(tVec(indx))];
    end
    cTime(iV) = deltaMat(iV,indx) + tVec(indx) + tVec(iV);
        
    while(indx <= nDim-nVehicles)
        indxB = find(vMatSolution(indx,:) == 1);
        if(indxB > nVehicles+nTasks)
            solStrA = [solStrA ' -> E' vecLetters(indxB-nVehicles-nTasks)];
            solStrB = [solStrB ' + ' num2str(deltaMat(indx,indxB)) ' + ' num2str(tVec(iV))];
            cTime(iV) = cTime(iV) + deltaMat(indx,indxB) + tVec(iV);
            solStrB = [solStrB ' = ' num2str(cTime(iV))];
        else
            solStrA = [solStrA ' -> ' taskLetters(indxB-nVehicles)];
            solStrB = [solStrB ' + ' num2str(deltaMat(indx,indxB)) ' + ' num2str(tVec(indxB))];
            cTime(iV) = cTime(iV) + deltaMat(indx,indxB) + tVec(indxB);
        end
        indx = indxB;
    end
end
solStrB = [solStrB ')'];

checkTime = max(cTime);
            [nR,nC] = size(vMatSolution);

            fprintf('This corresponds to the following routing:\n')
            fprintf('%s\n',solStrA);
            fprintf('\n %s = ',solStrB);
            fprintf('\n%.2f\n',checkTime);
            fprintf('\n');
            
             fprintf('Execution time = %.2f seconds\n',exTime);
           

end
end

fmt = [repmat('%4d ', 1, size(vMatBest,2)-1), '%4d\n'];
%fprintf(fmt, vMatBest.');   %transpose is important!
fprintf('\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

