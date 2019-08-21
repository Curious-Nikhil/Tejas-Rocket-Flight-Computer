close all

%% Initialize variables.
filename = 'C:\Users\Nikhil\Documents\GitHub\Tejas-Flight-Computer\Datadump\Tejas FC DropTest\8FL - 21-Aug.CSV';
delimiter = ',';

%% Read columns of data as text:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric text to numbers.
% Replace non-numeric text with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = mat2cell(dataArray{col}, ones(length(dataArray{col}), 1));
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[1,2,3,4,5,6,7]
    % Converts text in the input cell array to numbers. Replaced non-numeric
    % text with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1)
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData(row), regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if numbers.contains(',')
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(numbers, thousandsRegExp, 'once'))
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric text to numbers.
            if ~invalidThousandsSeparator
                numbers = textscan(char(strrep(numbers, ',', '')), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch
            raw{row, col} = rawData{row};
        end
    end
end


%% Replace non-numeric cells with NaN
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),raw); % Find non-numeric cells
raw(R) = {NaN}; % Replace non-numeric cells

%% Create output variable
FL21AUG = table;
FL21AUG.Time = cell2mat(raw(:, 1));
FL21AUG.Pascal = cell2mat(raw(:, 2));
FL21AUG.alt = cell2mat(raw(:, 3));
FL21AUG.KMF = cell2mat(raw(:, 4));
FL21AUG.ax = cell2mat(raw(:, 5));
FL21AUG.ay = cell2mat(raw(:, 6));
FL21AUG.az = cell2mat(raw(:, 7));


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans raw col numericData rawData row regexstr result numbers invalidThousandsSeparator thousandsRegExp R;

% figure
% 
% plot(FL21AUG.Time, FL21AUG.alt, FL21AUG.Time, FL21AUG.KMF, '--r')
% xlabel("Time");
% ylabel("Height");
% title("Kalman Filter");


figure

yyaxis left
plot(FL21AUG.Time, FL21AUG.alt, FL21AUG.Time, FL21AUG.KMF, '--k')

hold on
plot(FL21AUG.Time(384), FL21AUG.alt(384), 'r*', FL21AUG.Time(396), FL21AUG.alt(396), 'k*')

text(FL21AUG.Time(384), FL21AUG.alt(384),'\leftarrow Alt 4.79, 543m')
text(FL21AUG.Time(396), FL21AUG.KMF(396),'\leftarrow KMF 4.827, 538.56')

xlabel("Time");
ylabel("Height");
title("Kalman Filter");

yyaxis right
plot(FL21AUG.Time, FL21AUG.ax, 'r')
hold on
plot(FL21AUG.Time, FL21AUG.ay, 'g')
plot(FL21AUG.Time, FL21AUG.az, 'b')
hold off

ylabel("Accelerometer -x, y, z")
