%% Disclaimer :)
% It's also worth mentioning that sometimes that path generated isn't the
% most optimal due to the no. points, combniation of methods, etc. set(i think) 
% but it seems you just need to increase no. points or mix up the methods to 
% arrive at a pretty good path.
check_map;

map=imbinarize(imread('random_map.bmp')); %reading the map using im2bw (matlab recommended imbinarize)

%% partial handle to faulty maps
one = sum(map(:) == 1);
zero = sum(map(:) == 0);

% if obstacles are more than free space, likely to be faulty map
while(one < zero)
    create_image;
    map=imbinarize(imread('random_map.bmp'));

    one = sum(map(:) == 1);
    zero = sum(map(:) == 0);
end

% %show the randomly generated image
% %without the path
figure(2);
im = subplot(1, 1, 1);
imshow(map), title('Map image');
set(im, 'box', 'on', 'Visible', 'on', 'xtick', [], 'ytick', [])

start = [1,1];
finish = [500,500];

noOfPointsInSolution = 20; % no of points wanted in a solution, max tested is 50 points, but takes 2 mins, lower no of points may not give the most optimal path i.e., 10
noOfCoords = noOfPointsInSolution * 2; % an int var that accounts for number of x,y (i.e., for point 1 [x1,y1] )
fitness_score_col = noOfCoords + 1; % index for the fitness column in the population
iter = 150;% number of generations, set as wanted, more generations mean more routes is explored
population_size = 500; % number of chromosomes in population, set as wanted, more chromosomes also mean more diverse population
crossover_chance = 0.8; % chance for crossover to happen
mutation_chance = 0.3;  % chance for mutation to happen
collision_penalty = 2; % the exponential rate at which the penalty increases by the no. collisions had between 2 points

population = zeros(population_size, noOfCoords); % pre-allocate array for efficiency

%%% GETTING PARAMETERS

fprintf('\nAvailable selection method: \n 1. Roulette wheel \n 2. Tournament \n 3. Rank-based');
selection_method = input('\nEnter your choice:');

fprintf('\nAvailable crossover methods: \n 1. k-point \n 2. Uniform ');
crossover_method = input('\nEnter your choice:');

fprintf('\nAvailable mutation methods: \n 1. Bit-flip \n 2. Bit-swap');
mutation_method = input('\nEnter your choice:');

%%% GENETIC ALGORITHM

% Step 1.Generates a random population, with some intialisation to make
% sure each point progress after one another. 
for i = 1:population_size
    chromosome = zeros(1, noOfCoords);
    
    % calculate the proportion/ range of x & y in which the point is going
    % to take at most, where the range is simply the map size (500)/ number of
    % points wanted. i.e., for the first point, max_start = 1, max_end = 51
    % for number of points = 10.
    increment = round(500/noOfPointsInSolution);
    max_start = 1;
    max_end = max_start + increment - 1;

    % for abritrary and large no. points such as 30, the increment results in an odd
    % stepping of increases, and at the same time reduce the search space
    % significantly. Hence max_attempts of search for a non-obstacle point
    % is created. More explanation below.
    max_attempts = 50;

    % Then, we use this range of x & y to pick a random point for each
    % point. 
    for j = 1:noOfPointsInSolution
        % % handles odd case for number of points i.e., 30, increment would
        % be 17, resulting in out of bound max_end.
        if j == noOfPointsInSolution
            max_end = 500;
        end
        x = randi([round(max_start), round(max_end)]);
        y = randi([round(max_start), round(max_end)]);

        % if the point is an obstacle, redo
        attempt = 0;
        while map(y, x) == 0 && attempt < max_attempts
            x = randi([round(max_start), round(max_end)]);
            y = randi([round(max_start), round(max_end)]);
            attempt = attempt + 1;
        end
            
        % more than 50 attemps indicates that the search space is tight,
        % and seems to be fully occupied with an obstacle. Then, we round
        % the max_start down to the nearest 10th i.e., 405 -> 400 and round
        % max_end up to nearest 10th. This expansion of search space would
        % hopefully allow for vacant points. If not, default points should
        % be set.
        if attempt == max_attempts
            max_start_rounded = floor((max_start-1)/10)*10;
            max_end_rounded = ceil(max_end/10)*10;
            max_end_rounded = min(500, max_end_rounded);

            x = randi([round(max_start), round(max_end)]);
            y = randi([round(max_start), round(max_end)]);
    
            % if the point is an obstacle, redo
            attempt = 0;
            while map(y, x) == 0 && attempt < max_attempts
                x = randi([round(max_start), round(max_end)]);
                y = randi([round(max_start), round(max_end)]);
                attempt = attempt + 1;
            end
        end

        % Then insert these into the chromosome
        index = (j-1) * 2 + 1;
        chromosome(index) = x;
        chromosome(index + 1) = y;
        
        % update the maximum start and end for the next point/ part of the
        % map
        max_start = max_end;
        max_end = max_start + increment;

    end
    % By using the above initilisation, we make sure that each point
    % follow a steady progression towards the goal, rather than just
    % generating a sequence of random points on the map
    population(i, :) = chromosome;
end


%% extra column at the end for fitness scores
population = [population zeros(population_size,1)];
fittest = zeros(iter, 1); %initialize vector to store fitness score of fittest individual each generation for plotting

% create a progress for monitoring the progress during runtime
h = waitbar(0, 'Sampling population for iter...');

tic;
% Step 2. Fitness function to evaluate how good chromosomes are
for k = 1:iter
    % calculate the cost of path in each chromosome
    for i = 1:population_size
        chromosome = population(i, 1:noOfCoords);
        % function explanation more below
        cost = computePathCost(chromosome, map, collision_penalty); 

        % update fitness score in the population matrix
        population(i, fitness_score_col) = cost;
    end
    % rearrange the matrix from smallest value to biggest based on the
    % fitness scores
    population = sortrows(population, fitness_score_col);

    % saving the fittest chromosome to the k iteration
    fittest(k,1) = population(end, fitness_score_col);
    
    % creating a new population, keeping the best 30% fittest chromosomes
    population_new = zeros(population_size, noOfCoords);
    population_new(1:(0.3*population_size),:) = population(population_size-(0.3*population_size-1):population_size,1:noOfCoords);
    population_new_num = (0.3*population_size);
    
    % repeat the above process again,include 30% chromosomes from the last population and add new ones
    while (population_new_num < population_size)
        %% selection operator, functions defined at the end with further explanation
        % selection_method choices: 1 for Roulette wheel, 2 for Tournament, 3 for Rank-based
        % fitness_score_col: just an index for the fitness column\
        % function returns an index within the population
        choice1 = selection(selection_method, population, fitness_score_col);
        choice2 = selection(selection_method, population, fitness_score_col);
        
        parent1 = population(choice1, 1:noOfCoords);
        parent2 = population(choice2, 1:noOfCoords);
        
        %% crossover operator, function defined at the end with further explanation
        % crossover_method choices: 1 for 1(k)-point crossover, 2 for uniform crossover
        % crossover_chance: the probability of performing the crossover, set as wanted
        [offspring1, offspring2] = crossover(crossover_method, parent1, parent2, crossover_chance); 

        %% mutation operator, function defined at the end with further explaination
        % mutation_chance: the probability of performing the mutation, set as wanted
        if (rand() < mutation_chance)
            offspring1 = mutation(mutation_method, offspring1);
        end

        if (rand() < mutation_chance)
            offspring2 = mutation(mutation_method, offspring2);
        end
        
        %% put in new population, add first new chromosome
        population_new_num = population_new_num + 1;
        population_new(population_new_num,:) = offspring1;
        % add second chromosome
        if (population_new_num < population_size)
            population_new_num = population_new_num + 1;
            population_new(population_new_num,:) = offspring2;
        end
    end
    %% replace the old population with the new population, recalculate fitness on next iter
    population(:,1:noOfCoords) = population_new;
    waitbar(k / iter, h, sprintf('Sampling population for iter %d of %d', k, iter));
end

%% at end: update new fitness score and rank them
% this part from the above GA, put here because the iteration ends before
% the ranking happens.
for i = 1:population_size
    chromosome = population(i, 1:noOfCoords);
    cost = computePathCost(chromosome, map, collision_penalty);
    population(i,fitness_score_col) = cost;    
end

population = sortrows(population,fitness_score_col);
solution = population(end, 1: noOfCoords);

% record the time taken for the path to converge
time_taken = toc;
disp("Time taken to converge:");
disp(time_taken);

% displaying the final best path 
path = [start; [solution(2:2:end)' solution(1:2:end)']; finish];
clf;
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));

clf;
imshow(map);
rectangle('position', [1 1 size(map)-1], 'edgecolor', 'k');
line(path(:,2), path(:,1)); 
title(["Most optimal path with " num2str(noOfPointsInSolution) " points"]);

% Plot the points for better visualisation
% x_coords = [start(2); solution(1:2:end)'; finish(2)];
% y_coords = [start(1); solution(2:2:end)'; finish(1)];

x_coords = solution(1:2:end);
y_coords = solution(2:2:end);

hold on;
plot(x_coords, y_coords, 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b'); % bigger dot for each points, better visualise the turns
hold off;

%% fitness function for evaluating the path
function cost = computePathCost(path, map, penalty)
    % The base level penalty increases exponentially with
    % exponential_increase var, where it's multiplied with the number of
    % collisions it has had for a path. The fitness is calculated by adding
    % dist & penalty together. This is a max function, hence the inverse
    % fitness.
    % - distance of the path from 1 point to another calculated using pdist2,
    % built-in function for euclidean distance

    basePenalty = 2;
    exponential_increase = penalty;
    totalCollisions = 0;
    dist = 0;
        
    x_coords = path(1:2:end);
    y_coords = path(2:2:end);

    for i = 1:length(x_coords) - 1
        dist = dist + pdist2([x_coords(i), y_coords(i)], [x_coords(i+1), y_coords(i+1)]);
        totalCollisions = totalCollisions + checkCollision(x_coords(i), y_coords(i), x_coords(i+1), y_coords(i+1), map);
    end

    if (totalCollisions > 0) 
        penalty = basePenalty + (exponential_increase ^ totalCollisions);
    else % if there's no collision
        penalty = 0;
    end

    cost = penalty + dist;

    if dist == 0
        % In rare case where the distance == 0 (i.e., x1 = x2 and y1 = y2),
        % assign a really high cost to discourage this path.
        cost = 1e6;
    else
        % Since the MatLab has a floating point limit, there should be a 
        % base constant added to make sure cost isn't surpassing this
        % limit, which is a result of a fitness score being too small.
        % i.e., there was a result of inf, as it was 0.666666...
        constant = 1e-6;
        cost = 1 / (cost + constant); %inverse fitness = smaller distance becomes bigger value, and this is a max function
    end
end

%% supporting function for fitnesss function
function totalCollisions = checkCollision(x1, y1, x2, y2, map)
    % Check if the line segment between (x1, y1) and (x2, y2) collides with obstacles
    totalCollisions = 0;
    
    % utilise the bresenham algorithm on the MatLab page
    [x, y] = bresenham(x1, y1, x2, y2);

    for i = 1:length(x)
        % bresenham algo gives double type, which needs to be int for index
        current_x = round(x(i));
        current_y = round(y(i));

        % check if the current coord == 0 (an obstacle)
        if map(current_y, current_x) == 0
            totalCollisions = totalCollisions + 1;
        end
    end
    % totalCollisions
end

%% selection function, 3 methods
function choice = selection(selection_method, population, fitness_score_col)
    % Each time returns an index of the chosen chromosome
    if (selection_method == 1)
        % Roulette Wheel selection, where each fitness score is the
        % probability of being selected i.e., higher fitness higher
        % probability
        weights = population(:, fitness_score_col)/sum(population(:,fitness_score_col));
        accumulation = cumsum(weights);
        p = rand();
        chosen_index = -1;

        for index = 1 : length(accumulation)
            if (accumulation(index) > p)
                chosen_index = index;
                break;
            end
        end
        choice = chosen_index;

    elseif (selection_method == 2)
        % Tournament selection, here we set the number of tournaments held
        % to an arbritrary number. The higher number of tournament, the
        % more likely the best chromosome is selected
        noOfTournaments = 5;
        tournament_size = 3; % number of contestants to be competing

        for i=1:noOfTournaments
            %create a random indexing vector of population, then get
            %the fittest chromosome out of these using max().
            contestants_indicies = randi(size(population,1), 1, tournament_size);
            contestants_fitness = population(contestants_indicies, end);

            [~, fittest] = max(contestants_fitness);
            choice = contestants_indicies(fittest);
        end
    elseif (selection_method == 3)
        % Rank-based selection, where chromosomes are ranked based on 
        % their fitness values, with the lowest fitness valued one rank 
        % 1, and highest rank N.

        % this is linear ranking, where the index are just linear to the
        % order of the chromosome (already sorted from small to big)
        ranks = 1:size(population, 1);
        rank_probabilities = ranks / sum(ranks);
        accumulation = cumsum(rank_probabilities);
        
        % then, use Roulette wheel selection to pick the index.
        p = rand();
        chosen_index = -1;
        
        for index = 1:length(accumulation)
            if (accumulation(index) > p)
                chosen_index = index;
                break;
            end
        end
        
        choice = chosen_index;
    else 
        error('Unrecognised method')
    end 
end

%% crossover function, 2 methods
function [offspring1, offspring2] = crossover(crossover_method, parent1, parent2, p_c)
    if (crossover_method == 1)
        % k-point crossover of 1 point, with p_c being the
        % probability of whehter the recombination is performed or not
        if (rand() < p_c)
            k = 1; % Number of crossover points, for clarity
            % the below line creates a matrix of size [1, 2], specifying
            % the half to swap
            crossover_points = sort(randi([1, size(parent1,2)], 1, k));
            
            % here, the offsprings are created by swapping parents at crossover points
            offspring1 = parent1;
            offspring1(crossover_points:end) = parent2(crossover_points:end);
            
            offspring2 = parent2;
            offspring2(crossover_points:end) = parent1(crossover_points:end);
        else
            % don't crossover of rand is lower
            offspring1 = parent1;
            offspring2 = parent2;
        end
    elseif (crossover_method == 2)
       % uniform crossover
        % create a random vector of the parent chromosome size, since this
        % probability is perform on each allele, the condition is performed
        % on the vector itself
        rand_vec = rand(size(parent1)) >= 0.5;
        
        % set the offsprings to the original parents, then swap this value
        % with parent 
        offspring1 = parent1;
        offspring2 = parent2;

        offspring1(rand_vec) = parent2(rand_vec);
        offspring2(rand_vec) = parent1(rand_vec);
    else 
        error('Unrecognised method')
    end
end

%% mutation function, 2 methods
function flippedChromosome = mutation(mutation_method, chromosome)
    if (mutation_method == 1)
        % Bit-flip mutation
        point1 = randi([1, length(chromosome)]);
        point2 = randi([1, length(chromosome)]);
        
        while(point2 == point1)
            point2 = randi([1, length(chromosome)]); %so that point 1 != point2
        end

        if(point2 < point1) %swap values so that point1 < point2
            [point1, point2] = swap(point1, point2);
        end
        tobeflipped = chromosome(point1:point2);
        chromosome(point1:point2) = fliplr(tobeflipped);
        flippedChromosome = chromosome;

    elseif (mutation_method == 2)
        % Bit-swap mutation
        point1 = randi([1, length(chromosome)]);
        point2 = randi([1, length(chromosome)]);

        while(point2 == point1)
            point2 = randi([1, length(chromosome)]); % point1 != point2
        end

        % Swap the 2 values at point1 and point2
        temp = chromosome(point1);
        chromosome(point1) = chromosome(point2);
        chromosome(point2) = temp;
        flippedChromosome = chromosome;

    else 
        error('Unrecognised method')        
    end 
end

%% line drawing function, used to obtain all points of the line between 2 given points
% Original author: https://uk.mathworks.com/matlabcentral/fileexchange/28190-bresenham-optimized-for-matlab
function [x, y]=bresenham(x1,y1,x2,y2)
    %Matlab optmized version of Bresenham line algorithm. No loops.
    %Format:
    %               [x y]=bham(x1,y1,x2,y2)
    %
    %Input:
    %               (x1,y1): Start position
    %               (x2,y2): End position
    %
    %Output:
    %               x y: the line coordinates from (x1,y1) to (x2,y2)
    %
    %Usage example:
    %               [x y]=bham(1,1, 10,-5);
    %               plot(x,y,'or');
    x1=round(x1); x2=round(x2);
    y1=round(y1); y2=round(y2);
    dx=abs(x2-x1);
    dy=abs(y2-y1);
    steep=abs(dy)>abs(dx);
    if steep 
        t=dx;
        dx=dy;
        dy=t; 
    end
    
    %The main algorithm goes here.
    if dy==0 
        q=zeros(dx+1,1);
    else
        q=[0;diff(mod((floor(dx/2):-dy:-dy*dx+floor(dx/2))',dx))>=0];
    end
    
    %and ends here.
    
    if steep
        if y1<=y2 
            y=(y1:y2)'; 
        else 
            y=(y1:-1:y2)'; 
        end

        if x1<=x2 
            x=x1+cumsum(q);
        else 
            x=x1-cumsum(q); 
        end
    else
        if x1<=x2 
            x=(x1:x2)'; 
        else 
            x=(x1:-1:x2)'; 
        end
        if y1<=y2 
            y=y1+cumsum(q);
        else 
            y=y1-cumsum(q); 
        end
    end
end

% simple swap function
function [b, a] = swap(a, b)
end

function create_image
% MATLAB script to generate a binary image with circles, squares, rectangles, and triangles

% Set the size of the image
imageSize = 500;

% Create a blank white image
binaryImage = ones(imageSize);

% Number of random shapes
numShapes = 30;

% Generate random shapes
for i = 1:numShapes
    % Random shape type: 1 for rectangle, 2 for circle, 3 for triangle
    shapeType = randi([1, 3]);

    % Random position
    posX = randi([1, imageSize]);
    posY = randi([1, imageSize]);

    % Random size
    sizeX = randi([20, 50]); % Adjust the range as needed
    sizeY = randi([20, 50]); % Adjust the range as needed

    % Add the shape to the image
    if shapeType == 1
        % Rectangle
        binaryImage(posY:posY+sizeY, posX:posX+sizeX) = 0;
    elseif shapeType == 2
        % Circle
        [X, Y] = meshgrid(1:imageSize);
        mask = (X - posX).^2 + (Y - posY).^2 <= (sizeX / 2)^2;
        binaryImage(mask) = 0;
    else
        % Triangle
        triangle = poly2mask([posX, posX+sizeX, posX+sizeX/2], ...
                             [posY, posY, posY+sizeY], imageSize, imageSize);
        binaryImage(triangle) = 0;
    end
end

binaryImage = binaryImage(1:imageSize,1:imageSize);

% Ensure there is a clear path from (1,1) to (500,500)
binaryImage(imageSize-10:imageSize, 1:imageSize) = 1;
binaryImage(1:imageSize, 1:10) = 1;

% Display the generated image
imshow(binaryImage);
title('Binary Image with Random Shapes and a Clear Path');

% Save the image to a file if needed
imwrite(binaryImage, 'random_map.bmp');
end


function check_map
fprintf('\n Checking map...');
if exist('random_map.bmp', 'file') == 2
    % File exists
    fprintf('\n Found an existing map image. Would you like to use the same image? \n 1. Yes \n 2. No, create a new one');
    map_choice = input('\nEnter your choice:');
    if(map_choice == 1)
        return
    else
        create_image;
    end
else
    % File does not exist
    fprintf('No existing map image found. Creating a new one...');
    create_image; % call the provided script to create an image each run, comment to use the same image
end
end
