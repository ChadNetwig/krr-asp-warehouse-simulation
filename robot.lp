%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% filename           : robot.lp
% answer set program : Automated Warehouse Scenario
% code type          : clingo logic program
% author             : Chad Netwig
% date               : February 24, 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%sets maximum time steps for robotic movement
#const maxstep=15.

% valid robot movements for delta x column (DX) and delta y row (DY)
move(1,0).    % move right one column DX + 1
move(-1,0).   % move left one column DX - 1
move(0,1).    % move up one row DY + 1
move(0,-1).   % move down one row DY - 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build warehouse objects from "inits" based on ASP Challenge formatting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: Test inits are hardcoded at bottom of code for testing a 3x3 grid 
%     : warehouse. Comment out test environment to test the ASP Challenge
%     : instx.asp files
% 9-nodes
% 1-robot
% 3-highway cells
% 1-picking station
% 1-shelf
% 1-product
% 1-line order
% 1-pickingStation

% node object (builds a unique node for each cell in the warehouse grid)
node(N,X,Y) :- init(object(node,N),value(at,pair(X,Y))).

% robot location object at time 0
robotLoc(R,N,T) :- init(object(robot,R),value(at,pair(X,Y))), node(N,X,Y), T=0.

% highway location object
highwayLoc(H,N) :- init(object(highway,H),value(at,pair(X,Y))), node(N,X,Y).

% pickingStation location object
pickingStationLoc(P,N) :- init(object(pickingStation,P),value(at,pair(X,Y))), node(N,X,Y).

% shelf location object at time 0 - no robot initially carrying shelf R=0
shelfLoc(S,N,R,T) :- init(object(shelf,S),value(at,pair(X,Y))), node(N,X,Y), R=0, T=0.

% product units on a given shelf object information
productShelfUnits(I,S,U) :- init(object(product,I),value(on,pair(S,U))).

% order number and product number and units of that product
orderProductUnits(O,I,U) :- init(object(order,O),value(line,pair(I,U))).

% order number to a particular pickingStation
orderToPs(O,P) :- init(object(order,O),value(pickingStation,P)).

% build maximums for objects
maxCols(MC) :- MC=#max{X: node(N,X,Y)}.
maxRows(MR) :- MR=#max{Y: node(N,X,Y)}.
maxBots(MB) :- MB=#max{R: robotLoc(R,N,T)}.

% delivery goal
:- not deliverOrder(R,O,P,T), orderToPs(O,P), orderProductUnits(O,I,U), pickingStationLoc(P,N), robotLoc(R,N,T).

% optimize actions on "occurs"
#minimize{1,Object,Value,Time : occurs(Object,Value,Time)}.

% maxMoves required to complete order deliveries
maxMoves(N) :- N = #max{Time : occurs(Object,Value,Time)}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state description per ASP Challenge constraint requirements
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robots cannot swap spaces on any given time step
:- robotLoc(R1,N1,T), robotLoc(R1,N2,T+1), robotLoc(R2,N2,T), robotLoc(R2,N1,T+1), R1!=R2.

% no two robots can be on the same node at any time step
:- 2{robotLoc(R,N,T) : node(N,X,Y)}, robotLoc(R,_,_), T=0..maxstep.

% robot can exist on only one node at any time step
:- 2{robotLoc(R,N,T) : robotLoc(R,_,_)}, node(N,X,Y), T=0..maxstep.
%:- 2{robotLoc(R,N,T) : node(N,X,Y)}, T=0..maxstep.


% no picking station location can be on a highway node
:- pickingStationLoc(P,N), highwayLoc(H,N), node(N,X,Y).

% no shelf location can be on a highway node
:- shelfLoc(S,N,R,T), highwayLoc(H,N), node(N,X,Y), R=0..MB, maxBots(MB), T=0..maxstep.
%:- shelfLoc(S,N,R,T), highwayLoc(H,N), node(N,X,Y), R=1..MB, maxBots(MB), T=0..maxstep.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% domain independent axioms/exogenous actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% builds all possible robot moves for total number of robots over total max steps. Actions are exogenous
{moveBot(R,move(DX,DY),T) : move(DX,DY)}=1 :- R=1..MB, maxBots(MB), T=0..maxstep-1.

{pickupShelf(R,S,T) : shelfLoc(S,_,_,_)}=1 :- R=1..MB, maxBots(MB), T=0..maxstep-1.

{putdownShelf(R,S,T) : shelfLoc(S,_,_,_)}=1 :- R=1..MB, maxBots(MB), T=0..maxstep-1.

{deliverOrder(R,O,P,T) : orderToPs(O,P)}=1 :- R=1..MB, maxBots(MB), T=0..maxstep-1.

% commonsense law of intertia
% by default, robot(s) will stay on same node through time steps unless explicitly moved
{robotLoc(R,N,T+1)} :- robotLoc(R,N,T), T < maxstep.

% by default, shelves will stay on same node through time steps unless explicitly moved by robot(s)
{shelfLoc(S,N,R,T+1)} :- shelfLoc(S,N,R,T), T < maxstep.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% effect and preconditions of actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% direct effect of actions

% moving robot effect on robots' location node change based on delta x and delta y move
robotLoc(R,N1,T+1) :- robotLoc(R,N,T), moveBot(R,move(DX,DY),T), node(N,X,Y), node(N1,X+DX,Y+DY).

% direct effect of robot picking up a shelf
shelfLoc(S,N,R,T+1) :- shelfLoc(S,N,R,T), robotLoc(R,N,T), pickupShelf(R,S,T).

% direct effect of robot putting down a shelf - setting Robot to zero means robot is no longer carrying the shelf
shelfLoc(S,N,R1,T+1) :- shelfLoc(S,N,R,T), robotLoc(R,N,T), putdownShelf(R,S,T), R1=0.

% preconditions of actions
% constrains robot from moving off the lower bounds of the grid when X and or Y are less than 1
:- robotLoc(R,N,T), moveBot(R,move(DX,DY),T), node(N,X,Y), X+DX<1.
:- robotLoc(R,N,T), moveBot(R,move(DX,DY),T), node(N,X,Y), Y+DY<1.

% constrains robot from moving over the upper bounds of the grid based on maximum columns and rows
:- robotLoc(R,N,T), moveBot(R,move(DX,DY),T), node(N,X,Y), maxCols(MC), X+DX>MC.
:- robotLoc(R,N,T), moveBot(R,move(DX,DY),T), node(N,X,Y), maxRows(RC), Y+DY>RC.
%:- robotLoc(R,N,T), moveBot(R,move(DX,DY),T), node(N,X,Y), X+DX>3.
%:- robotLoc(R,N,T), moveBot(R,move(DX,DY),T), node(N,X,Y), Y+DY>3.

% robot must be a pickingStation node to deliver
%:- deliverOrder(R,O,P,T), pickingStationLoc(P,N), not robotLoc(R,N,T).

% no shelf can be putdown on a  highway node
%:- putdownShelf(R,S,T), robotLoc(R,N,T), highwayLoc(H,N).

% robot must be at the shelf location in order to pick up the shelf
:- pickupShelf(R,S,T), shelfLoc(S,N,R,T), not robotLoc(R,N,T).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% format actions to ASP Challenge solutions output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
occurs(object(robot,R),move(DX,DY),T) :- moveBot(R,move(DX,DY),T).

occurs(object(robot,R),pickup,T) :- pickupShelf(R,S,T).

occurs(object(robot,R),putdown,T) :- putdownShelf(R,S,T).

occurs(object(robot,R),deliver(O,I,U),T) :- deliverOrder(R,O,P,T), orderProductUnits(O,I,U).


%%%%%%%%%%%%%%%%%%%%%%%
% shows for diagnostics
%%%%%%%%%%%%%%%%%%%%%%%

% counts
%#show maxRows/1.

% objects from inits
%#show node/3.
%#show robotLoc/3.
%#show highwayLoc/2.
%#show pickingStationLoc/2.
%#show shelfLoc/4.
%#show productShelfUnits/3.
%#show orderProductUnits/3.
%#show orderToPs/2.

% actions
%#show moveBot/3.
%#show pickupShelf/3.
%#show putdownShelf/3.
%#show deliverOrder/4.

% ASP Challenge formatted output
#show occurs/3.
#show maxMoves/1.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% test input "init" for 3x3 warehouse
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% establish a 3x3 test warehouse with 9 nodes populated as follows:
% 1-robot
% 3-highway cells
% 1-picking station
% 1-shelf
% 1-product
% 1-line order
% 1-pickingStation

% node initialization: init(object(node,N),value(at,pair(X,Y)))
% where N = node number and X,Y = a grid cell
% sets up 9 nodes, one for each X,Y grid cell pair
%init(object(node,1),value(at,pair(1,1))).
%init(object(node,2),value(at,pair(1,2))).
%init(object(node,3),value(at,pair(1,3))).
%init(object(node,4),value(at,pair(2,1))).
%init(object(node,5),value(at,pair(2,2))).
%init(object(node,6),value(at,pair(2,3))).
%init(object(node,7),value(at,pair(3,1))).
%init(object(node,8),value(at,pair(3,2))).
%init(object(node,9),value(at,pair(3,3))).


% robot initialization: init(object(robot,R),value(at,pair(X,Y))), where R = robot number and X,Y = a grid cell
% sets up one robot with initial postion of grid cell (1,1)
%init(object(robot,1),value(at,pair(1,1))).
%init(object(robot,2),value(at,pair(3,3))).


% highway initialization: init(object(highway,H),value(at,pair(X,Y)))
% where H = highway cell and X,Y = a grid cell
% sets up 3 highway cells on top row
%init(object(highway,1),value(at,pair(1,1))).
%init(object(highway,2),value(at,pair(2,1))).
%init(object(highway,3),value(at,pair(3,1))).


% pickingStation initialization: init(object(pickingStation,P),value(at,pair(X,Y)))
% where P = pickingStation cell and X,Y = a grid cell
% sets up 1 picking station at fixed cell (1,3)
%init(object(pickingStation,1),value(at,pair(1,3))).
%init(object(pickingStation,2),value(at,pair(1,1))).


% shelf initialization: init(object(shelf,S),value(at,pair(X,Y)))
% where S = shelf cell and X,Y = a grid cell
% sets up 1 picking shelf at fixed cell (2,2)
%init(object(shelf,1),value(at,pair(2,2))).


% product initialization: init(object(product,I),value(on,pair(S,U)))
% where I = product label, on S = shelf location, U = number of units of that product
% sets up product #1 on shelf 1 with 6 units
%init(object(product,1),value(on,pair(1,6))).


% order initialization: init(object(order,O),value(line,pair(I,U)))
% where O = order label, I = product label, and U = number of units of that product
% sets up line order for order #1, product #1, and 4  units of product #1
%init(object(order,1),value(line,pair(1,4))).


% deliverOrderOrdery initialization: init(object(order,O),value(pickingStation,P))
% where O = order label, P = pickingStation number
% sets up order #1 to be deliverOrderOrdered to pickingStation #1
%init(object(order,1),value(pickingStation,1)).
