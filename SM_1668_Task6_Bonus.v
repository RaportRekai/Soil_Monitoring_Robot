/*
*Team Id: 1668
*Author List: Dan Mani Binu, Nandakishore MN, Sree Sankar TM, Anupam Kurien Mathew
*File Name: SM_1668_Task6_Bonus
*Theme: Soil Monitoring Bot
*/

module SM_1668_Task6_Bonus(
	input  clk_50,	
	input in,
	input  dout,				
	output adc_cs_n,			
	output din,					
	output adc_sck,			
	output [11:0]d_out_ch5,	
	output [11:0]d_out_ch6,	
	output [11:0]d_out_ch7,	
	output [1:0]data_frame,
	
	output A_1a,   
	output A_1b, 
	output B_1a,
	output B_1b,
	
	output tx,
	
   output r1, // red color to led1
	output g1, // green color to led1
	output b1, // blue color to led1
	
	output PWM_OUT,
	
	output reg s2, //to select color filter to be used
	output reg s3, //to select color filter to be used
	
	
	output wire emout, //turns on electromagnet
	
	output r2, // red color to led2
	output g2, // green color to led2
	output b2, // blue color to led2
	output r3, // red color to led2
	output b3, // blue color to led2
	output g3  // green color to led2

);
	
//7 stores output of 5, 5 stores output of 6, 6 stores output of 7
//left sensor 7 right sensor 5 centre sensor 6
// if right sensor is detected rotate left motor
//if left sensor is detected rotate right motor
//if center is detected 2 motors working
//B controls right wheel
//A controls left wheel
//right sensor 5 detected the make B 0,0 and A 1,0
//left sensor 7 detected then make A 0,0 and B 1,0
//If center 6 is detected make A and B 1,0
//if none is true then 0,0 for both
//GPIO10 LV3 A_1a
//GPIO11 LV4 A_1b
//GPIO12 LV2 B_1b
//GPIO13 LV1 B_1a

reg[8:0] s;                     //source and target to be fed into djikstras
reg[8:0] target;

reg[8:0]c_node;                 //stores the node number at which bot stands
reg[8:0]n_node;                 //stores the next node number at which bot goes to
reg [8:0]p_node;					  //stores previous node where bot was at

reg [31:0]color_timer;          // create delay before the same color is detected again 

reg [47:0]Str;                  // store string of message during supply and deposition
reg [39:0]Str1;                 // store string of message during color detection
reg [31:0]C;                    // store the requirement needed in each field for UART
reg [15:0]DZP;                  // store the position of deposition
reg c_d_p;							  // turns on when the bot goes for color detection in paddy plane
reg c_d_n;                      // turns on when the bot goes for color detection in nutty ground
reg c_d_v;                      // turns on when the bot goes for color detection in vegetable garden
reg c_d_m;						     // turns on when the bot goes for color detection in maize terrain

reg detect_run_p;               // turns on when the bot goes for run in paddy plane
reg detect_run_v;               // turns on when the bot goes for run in vegetable garden
reg detect_run_m;               // turns on when the bot goes for run in maize terrain
reg detect_run_ng=1;				  // turns on when the bot goes for run in nutty ground

reg [31:0]g_c;                  // stores no.of times green is detected in the mentioned range of color sensor
reg [31:0]b_c;
reg [31:0]r_c;

reg drop_spot_0;                // flag variables used to prevent control flow from reentering green deposition zone once again
reg drop_spot_1;                // flag variables used to prevent control flow from reentering green deposition zone once again
reg drop_spot_2;                // flag variables used to prevent control flow from reentering green deposition zone once again


reg visited_9;                  // flag to know whether node 9 was visited once or not

reg g_stop=0;                   // flag variables used to prevent control flow from reentering green deposition zone once again  
reg b_stop=0;                   // flag variables used to prevent control flow from reentering blue deposition zone once again  
reg r_stop=0;                   // flag variables used to prevent control flow from reentering red deposition zone once again

reg [1:0]drop_color[0:2];       // drop_color[0] stores color to be deposited at first detected SI similiarly 
reg [8:0]drop_spot[0:2];        // similiar to drop_color, stores drop position   
reg [8:0]drop_spot_c[0:2];      // copy of drop_spot is taken

reg [1:0]d_k=0;                 // deposition counter, counts the no. of deposits in a field



reg [20:0]f_count;              // to obtain output from color sensor

reg [7:0] cntnode;
reg [8:0]k=0;
reg breaker;                    // flag which helps to break the turn when bot turns 90 or 180 degree 
reg [31:0] sec;                 // seconds counter to help bot turn through 90 degree 
reg t_emout = 1;                // reg variable attached to emout
assign emout = t_emout;         
reg[5:0]ven;                    // used to reverse djkstras array

////////////////////////////////Variables for Djkstras Algorithm/////////////////////////////////

reg [26:0]count_dj=0;           // To reduce the number of conditions in finite state machine
integer t;

reg [3:0]dj_d[1368:0];          // To store distance between nodes in djkstras array
reg dj_e[1368:0];               // array used to store whether a line exists between two nodes in the arena
reg [6:0] u,v,a_dj,b_dj;        // used to iterate through the djkstras algorithm
reg[6:0]i_dj;
reg visited[0:36];              // to store the visited nodes in an array in djkstras graph
reg [7:0]distance[0:36];        // stores the calculated distance between the source and various other nodes
reg [6:0]nextvlist[0:36];       // stores the node numbers of those nodes having same distance from one node to another (aids in djkstras algorithm)

integer k_dj;
integer count_u_dj;             // since there are 37 nodes in graph, djkstras algorithm returns distances between all the nodes from source node
reg [1:0]flag_dj=0;             // to track end of djkstras algorithm calculation    
reg [6:0]nextv;                 // next vertex to be visited for djkstras algorithm distance calculation
reg [7:0]nextd;                 // distance to the next vertex (node)
reg [6:0]store[0:37];           // to store the nodes to be followed to reach destination
reg [6:0]store_final[0:37];     // to store reversed node

reg [6:0]greenpick[0:1];        // to store position where green block is present
reg [6:0]bluepick[0:1];         // to store position where blue block is present
reg [6:0]redpick[0:1];          // similiarly 

reg r_k=0;                      // keeps track of the red blocks picked
reg g_k=0;                      // keeps track of green blocks picked
reg b_k=0;                      // keeps track of blue blocks picked

reg gamechanger=0;              // The Flag Variable to update the source and target nodes of djkstras algorithm
reg flag_game=0;                // flag used to change gamechanger to zero
reg flag_ven=0;
reg [5:0]z;
/////////////////////////////////////////////////////////////////

reg [31:0] sec_c;               // seconds counter for color 
reg [31:0] secu;                // seconds counter for u turn
reg [31:0]delay;                // sets delay for color detection      

reg [31:0]ccount;               // variables used as pwm regulators
wire [31:0]coun;                // similar
reg strt_delay;                 // flag to start delay for next color to be detected
reg [16:0]uart;                 // to count bits being send for message transfer

reg red_c=0;                    // to initialize 
reg green_c=0;
reg blue_c=0;

reg [7:0]A[0:36];               //edge weight matrix used to store the graph for dijkstra's algorithm
reg nkmn;

reg flagturnleft;               //flag gets set when the bot turns left and reset when normal
reg flagturnright;              //flag gets set when the bot turns right and reset when normal
reg detect;                     //to check whether node is detected
reg flagnode=0;                 //flag gets set when the bot detects a node and reset when normal
reg [31:0] timer;
reg flaguturn=0;                //flag gets set when the bot takes u turn and reset when normal
reg flag_emon;                  //flag gets set when the electromagnet is on and reset when off   
reg flag_supplystart;
reg flag_emoff;                 //flag gets set when the electromagnet is off and reset when on         
reg flagleft=0;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg[96:0]msg;                   // message to be sent
reg[2:0]c=2'b10;                // array as 0 in first and 1 in second index
reg[9:0]x=9'b0;                 // variables used for UART transmission message 
reg[2:0]state='d0;
reg[6:0]i='d0;
reg[1:0]j='b1;
reg[1:0]q='d0;//Variable used to avoid time error.Also may helpfull in future while larger message transmission//
assign tx=j[0];
////////////////////////////////////////////////////////////////////////////////////////////////////////////
assign coun = ccount;
reg [7:0]d;
reg [7:0]pre_d;
reg [3:0]i_m;
reg [8:0]l_m;
reg [9:0]a_m;
reg [3:0]cntnode_m;
reg r_k_m;
reg b_k_m;
reg g_k_m;
reg[511:0]SPM = "SPM-Red-Green-Green-Blue-Blue-Red-Null-#";

////////////////////////////////////INITIAL BLOCK FOR ASSIGNING PATH GRAPH////////////////////////////////////////////////////////////////////////
initial
begin
Str="";
d="";
pre_d="";                        //The Initial block is used to initialize the variables declared mentioned above
detect_run_p=0;
c_d_p=0;
d_k=0;
c_d_m=0;
drop_spot_c[0]=0;
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
drop_spot_0=0;
drop_spot_1=0;
drop_spot_2=0;
visited_9=0;
c_d_n=1;
c_d_v=0;
p_node=1;
c_node=0;
n_node=19;
k=1;
detect_run_v=0;
detect_run_m=0;
s=0;
target=36;

i_m=0;// iterator for SPM preocessing
l_m=511;// no.of bits in SPM
cntnode_m=0;// node count for SPM
a_m=511; // iterator for SPM preocessing
end

///////////////////////////////////////////////////CONTROL SPEED OF THE BOT/////////////////////////////////////////////////////////

always @(posedge clk_50) // PWM module
begin
if(ccount< 50000)
ccount <= ccount + 1'b1;
else 
ccount <= 0;
end
integer c_max = 98*500; // The 98 reffered here sets the bot to 98 percent speed
assign PWM_OUT = (coun<c_max?1'b1:1'b0);
///////////////////////////////////////////////VARIABLES FOR COLOR DETECTION/////////////////////////////////////////////
reg [20:0]count;
reg t_r1,t_b1,t_g1,t_r2,t_b2,t_g2,t_r3,t_b3,t_g3;   // assigning pins to colors of led1 led2 led3
assign r1= t_r1;
assign g1= t_g1;
assign b1= t_b1;
assign r2= t_r2;
assign g2= t_g2;
assign b2= t_b2;
assign r3= t_r3;
assign g3= t_g3;
assign b3= t_b3;
reg [31:0]blink_delayer;                           // delays blink
reg blink_delay;


//////////////////////////////////////////////VARIABLES FOR LINE SENSOR////////////////////////////////////////////////
reg flag_off=1;
reg flag1_1;
reg FLAG;
reg t_A_1a;                                   // reg variables for motor driver output 
reg t_A_1b;                                   // "
reg t_B_1a;                                   // "
reg t_B_1b;                                   // "
reg [2:0] ans;
reg [10:0]tot_count;
assign A_1a = t_A_1a & PWM_OUT;                //reg variable assigned to motor output driver wire
assign A_1b = t_A_1b & PWM_OUT;                //"
assign B_1a = t_B_1a & PWM_OUT;                //"
assign B_1b = t_B_1b & PWM_OUT;                //"
reg [11:0] t_d_out_ch7 = 12'b000000000000;     // initializing line sensor variables
reg [11:0] t_d_out_ch6 = 12'b000000000000;     //"
reg [11:0] t_d_out_ch5 = 12'b000000000000;     //"
reg [11:0] ctrl7 = 12'b000000000000;           //"
reg [11:0] ctrl6 = 12'b000000000000;           //"
reg [11:0] ctrl5 = 12'b000000000000;           //"
reg flag1 = 'b0;
integer a =-1;
reg start;
reg t_din = 1'b0; 
integer flag2 = 0;
integer count1 = 0;
integer count2 = 0;
integer cc=0;
reg cs_n = 1'b1;
reg cs_n_2 = 1'b0;
reg alt = 0;
reg flaglastlyt=0;
reg [31:0] lastlyt;
assign din = t_din;
assign d_out_ch7 =  t_d_out_ch7 & ctrl7;
assign d_out_ch6 =  t_d_out_ch6 & ctrl6;
assign d_out_ch5 =  t_d_out_ch5 & ctrl5;
assign adc_cs_n = alt?0:cs_n +  cs_n_2;


////////////////////////////////////////////CLOCK CONTROL FOR ELECTROMAGNET, COLOR SENSOR, MESSAGE DISPLAY/////////////////////

always @(negedge clk_50)
begin
if((c_node==18 || c_node==20 || c_node==15 || c_node==17 || c_node==12 || c_node==14 || c_node==11)&& t_emout==1)
begin
t_emout=0;
flag_emon=1;
end
if((c_node==25 || c_node==23 || c_node==27 || c_node==30 || c_node==35 || c_node==33 || c_node==8 || c_node==3 || c_node==6)&&t_emout==0)
begin
t_emout=1;
flag_emoff=1;
end
////////////////////////////SPM/////////

if(a_m>=0)
begin
if(l_m>=0)             
begin
d={d,SPM[l_m]};
i_m=i_m+1;
l_m=l_m-1;
if(i_m==7)
begin
i_m=0;
if(d=="-")
begin

cntnode_m=cntnode_m+1;
if(cntnode_m==1)
begin
 if(pre_d=="n")
 begin
 greenpick[g_k_m]=18;
 g_k_m=g_k_m+1;
 end
 if(pre_d=="e")
 begin
 bluepick[b_k_m]=18;
 b_k_m=b_k_m+1;
 end
  if(pre_d=="d")
 begin
 redpick[r_k_m]=18;
 r_k_m=r_k_m+1;
 end
end

if(cntnode_m==2)
begin
 if(pre_d=="n")
 begin
 greenpick[g_k_m]=20;
 g_k_m=g_k_m+1;
 end
 if(pre_d=="e")
 begin
 bluepick[b_k_m]=20;
 b_k_m=b_k_m+1;
 end
  if(pre_d=="d")
 begin
 redpick[r_k_m]=20;
 r_k_m=r_k_m+1;
 end
end

if(cntnode_m==3)
begin
 if(pre_d=="n")
 begin
 greenpick[g_k_m]=15;
 g_k_m=g_k_m+1;
 end
 if(pre_d=="e")
 begin
 bluepick[b_k_m]=15;
 b_k_m=b_k_m+1;
 end
  if(pre_d=="d")
 begin
 redpick[r_k_m]=15;
 r_k_m=r_k_m+1;
 end
end

if(cntnode_m==4)
begin
 if(pre_d=="n")
 begin
 greenpick[g_k_m]=17;
 g_k_m=g_k_m+1;
 end
 if(pre_d=="e")
 begin
 bluepick[b_k_m]=17;
 b_k_m=b_k_m+1;
 end
  if(pre_d=="d")
 begin
 redpick[r_k_m]=17;
 r_k_m=r_k_m+1;
 end
end

if(cntnode_m==5)
begin
 if(pre_d=="n")
 begin
 greenpick[g_k_m]=12;
 g_k_m=g_k_m+1;
 end
 if(pre_d=="e")
 begin
 bluepick[b_k_m]=12;
 b_k_m=b_k_m+1;
 end
  if(pre_d=="d")
 begin
 redpick[r_k_m]=12;
 r_k_m=r_k_m+1;
 end
end

if(cntnode_m==6)
begin
 if(pre_d=="n")
 begin
 greenpick[g_k_m]=14;
 g_k_m=g_k_m+1;
 end
 if(pre_d=="e")
 begin
 bluepick[b_k_m]=14;
 b_k_m=b_k_m+1;
 end
  if(pre_d=="d")
 begin
 redpick[r_k_m]=14;
 r_k_m=r_k_m+1;
 end
end

if(cntnode_m==7)
begin
 if(pre_d=="n")
 begin
 greenpick[g_k_m]=11;
 g_k_m=g_k_m+1;
 end
 if(pre_d=="e")
 begin
 bluepick[b_k_m]=11;
 b_k_m=b_k_m+1;
 end
  if(pre_d=="d")
 begin
 redpick[r_k_m]=11;
 r_k_m=r_k_m+1;
 end
end

end

pre_d=d;
d="";
end

end
a_m=a_m-1;
end


//////////////////////////////////////////BLINK LIGHTS AT END///////////////////////////////////////////

if((c_node==6 && drop_spot[1]==0)||c_node==3||(c_node==4 && drop_spot[0]==0))
flaglastlyt=1;
if(flaglastlyt==1 && lastlyt<100000000) //flaglastlyt starts creating a delay for the final blinkng of light 
lastlyt=lastlyt+1;
if(flaglastlyt==1 && lastlyt>0 && lastlyt<25000000) //turns on all red leds for 0.5 sec
begin
t_r1=1;
t_r2=1;
t_r3=1;
t_b1=0;
t_g1=0;
end
if(flaglastlyt==1 && lastlyt>25000000 && lastlyt<50000000) //turns off all red leds for 0.5 sec
begin
t_r1=0;
t_r2=0;
t_r3=0;
t_b1=0;
t_g1=0;
end
if(flaglastlyt==1 && lastlyt>50000000 && lastlyt<75000000) //again turns on all red leds for 0.5 sec
begin
t_r1=1;
t_r2=1;
t_r3=1;
t_b1=0;
t_g1=0;
end
if(flaglastlyt==1 && lastlyt>75000000 && lastlyt<100000000)//turns off all red led for 0.5 sec
begin
t_r1=0;
t_r2=0;
t_r3=0;
t_b1=0;
t_g1=0;
end
if(flaglastlyt==1 && lastlyt==100000000)
flaglastlyt=0;


///////////////////////////////////////////////////SUPPLY MESSAGE DISPLAY////////////////////////////////////////////////
if(uart<57725 && (flag_emoff==1||flag_emon==1||green_c==1||red_c==1||blue_c==1))
begin
if(uart==0) // if UART is intially 0 the message to be sent is broken down and concatenated by parts determining the deposition supply and drop zone
begin

if(c_node==35||c_node==30||c_node==33||c_node==8||c_node==27||c_node==25||c_node==23||c_node==6||c_node==3)// part of message to determine drop zone
begin
Str="ZD-D-S";
end

if(c_node==11||c_node==14||c_node==17||c_node==20||c_node==18||c_node==15||c_node==12)// part of message to determine supple zone
begin
Str="ZD-P-S";
end

if(green_c==1||red_c==1||blue_c==1)// part of message to determine status indicator
Str1="IS-IS";


if(drop_color[0]!=0) //chooses C corresponding to the Status Indicator
begin
if(drop_color[0]==1)
C="#-P-";
else if(drop_color[0]==2)
C="#-N-";
else if(drop_color[0]==3)
C="#-W-";
end

else if(drop_color[1]!=0)// chooses C corresponding to the Status Indicator
begin
if(drop_color[1]==1)
C="#-P-";
else if(drop_color[1]==2)
C="#-N-";
else if(drop_color[1]==3)
C="#-W-";
end

else if(drop_color[2]!=0)// chooses C corresponding to the Status Indicator
begin
if(drop_color[2]==1)
C="#-P-";
else if(drop_color[2]==2)
C="#-N-";
else if(drop_color[2]==3)
C="#-W-";
end

if(drop_spot_c[0]!=0) // chooses DZP corresponding to the deposition zone
begin
if(drop_spot_c[0]==35)
DZP="1N";
if(drop_spot_c[0]==30)
DZP="2N";
if(drop_spot_c[0]==33)
DZP="2V";
if(drop_spot_c[0]==8)
DZP="1V";
if(drop_spot_c[0]==27)
DZP="3M";
if(drop_spot_c[0]==25)
DZP="2M";
if(drop_spot_c[0]==23)
DZP="1M";
if(drop_spot_c[0]==6)
DZP="2P";
if(drop_spot_c[0]==3)
DZP="1P";
end

else if(drop_spot_c[1]!=0)// chooses DZP corresponding to the deposition zone
begin
if(drop_spot_c[1]==35)
DZP="1N";
if(drop_spot_c[1]==30)
DZP="2N";
if(drop_spot_c[1]==33)
DZP="2V";
if(drop_spot_c[1]==8)
DZP="1V";
if(drop_spot_c[1]==27)
DZP="3M";
if(drop_spot_c[1]==25)
DZP="2M";
if(drop_spot_c[1]==23)
DZP="1M";
if(drop_spot_c[1]==6)
DZP="2P";
if(drop_spot_c[1]==3)
DZP="1P";
end

else if(drop_spot_c[2]!=0) // chooses DZP corresponding to the deposition zone
begin
if(drop_spot_c[2]==35)
DZP="1N";
if(drop_spot_c[2]==30)
DZP="2N";
if(drop_spot_c[2]==33)
DZP="2V";
if(drop_spot_c[2]==8)
DZP="1V";
if(drop_spot_c[2]==27)
DZP="3M";
if(drop_spot_c[2]==25)
DZP="2M";
if(drop_spot_c[2]==23)
DZP="1M";
if(drop_spot_c[2]==6)
DZP="2P";
if(drop_spot_c[2]==3)
DZP="1P";
end

if(green_c==1||red_c==1||blue_c==1) // Strings are concatenated and msg is sent
msg={" ",C,DZP,Str1};               // the message format is different for deposition supply and status indication
else
msg={C,DZP,Str};

x=9'b0;
i='d0;
j='b1;
q='d0;
end


/////////////////////////////////////// Transmitting message using Xbee module
uart=uart+1;
x=x+1;
if(x==9'b110110011 && state==0 && (i<96)&&(q==0))
begin
j[0]=c[0];
q=q+1;
x=9'b0;
state=state+1;
end
else if(x==9'b110110010 && state==0 && (i<96)&&(q!=0))
begin
j[0]=c[0];
x=9'b0;
state=state+1;
end
else if (x==9'b110110010 && state==1)
begin
j[0]=msg[i];
x=9'b0;
i=i+1;
if(i%8==0)
begin
state=state+1;
end
end
else if (x==9'b110110010 && ((state==2)||(state==3)))
begin
j[0]=c[1];
x=9'b0;
state=state+1;
if(state==4)
begin
state=state-4;
end
end
/////////////////////////////////////////////////////////////////
end
if(uart==57725) // Once UART message has been sent all variable are reinitialized for next tranmission
begin
uart=0;
if(flag_emoff==1)
flag_emoff=0;
else if(flag_emon==1)
flag_emon=0;
else if(red_c==1)
red_c=0;
else if(green_c==1)
green_c=0;
else if(blue_c==1)
blue_c=0;
end


if(sec_c<15000000 && detect==0 )      // sec_c is used to partition the color sensor to red,blue and green filters
sec_c=sec_c+1;
if(sec_c == 15000000)                 // resets sec_c to 0 once the threshold values are reached 
begin
sec_c=0;
end
if(strt_delay==1 && delay<20000000)   // start delay turns 1 when one of the colors are detected and delay starts for 20000000 clock cycles
delay=delay+1;
if(delay == 20000000)
begin
delay=0;
strt_delay=0;
end
if(blink_delay==1 && blink_delayer<20000000)
blink_delayer=blink_delayer+1;
if(blink_delayer == 20000000)
begin
blink_delay=0;
blink_delayer=0;
flag_off=0;
end


///


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////OBTAIN OUTPUT FROM COLOR SENSOR///////////////////////////////////////
if(in==1)
begin
count=count+1;
end
if(in==0 && count!=0)
begin
f_count = count;
count = 0;
end
////////////////////////////////////////////////////////Turning on and turning off led


/////////////////////////Snippet 1///////////////////////////////////////////

if(drop_color[0]==1) //When the first SI is found in a given ground, this led1 glows
begin
t_r1=1;             // it is noted that if drop_color[n] == 1, then color detected is red
t_g1=0;
t_b1=0;
end
else if(drop_color[0]==2) // it is noted that if drop_color[n] == 2, then color detected is green
begin
t_g1=1;
t_r1=0;
t_b1=0;
end
else if(drop_color[0]==3) // it is noted that if drop_color[n] == 3, then color detected is blue
begin
t_b1=1;
t_g1=0;
t_r1=0;
end
/////////////////////////////////////////////////////////////////////////////////////////

if(drop_color[1]==1) // The same case is repeated for drop_color[1] which was earlier used for drop_color[0]
begin
t_r2=1;
t_g2=0;
t_b2=0;
end
else if(drop_color[1]==2) // similiar to Snippet 1
begin
t_g2=1;
t_r2=0;
t_b2=0;
end
else if(drop_color[1]==3)// similiar to Snippet 1
begin
t_b2=1;
t_r2=0;
t_g2=0;
end
///////////////////////////////////////////////////////////////////////////////////////////////

if(drop_color[2]==1)// similiar to Snippet 1
begin
t_r3=1;
t_g3=0;
t_b3=0;
end
else if(drop_color[2]==2)// similiar to Snippet 1
begin
t_g3=1;
t_r3=0;
t_b3=0;
end
else if(drop_color[2]==3)// similiar to Snippet 1
begin
t_b3=1;
t_g3=0;
t_r3=0;
end
//////////////////////////////////////////////////////////////////////////////////////////////


if(drop_spot_c[0]==c_node) // if the deposition node is reached as current node then led1 or led 2 or led 3 is switched off
begin
t_r1=0;
t_b1=0;
t_g1=0;
end

if(drop_spot_c[1]==c_node) //"
begin
t_r2=0;
t_b2=0;
t_g2=0;
end

if(drop_spot_c[2]==c_node) //"
begin
t_r3=0;
t_b3=0;
t_g3=0;
end

////////////////////////////////////COLOR DETECTION///////////////////////////////////////////////////////

///////////////////Snippet 2/////////////////////////


if(count==0 && strt_delay==0)   // Now sec_c is used to time partition the different color filters to be used
begin                           // 15000000 clock cycles are partiotioned equally for each filter once color nees to be detected
if(sec_c>0 && sec_c<5000000 )   // Time patition for red filter
begin
s2=0;                           // selecting red filter
s3=0;
if(f_count>11000 && f_count<13000  && (((c_node==34||c_node==29)&&c_d_n==1)||((c_node==32||c_node==7)&&c_d_v==1)||((c_node==26||c_node==24||c_node==22)&&c_d_m==1)||((c_node==5||c_node==2)&&c_d_p==1)) && detect==0)
begin                           // The f_count range is set and once the bot reches nodes before SI patches the color sensor starts detect
r_c=r_c+1;                      // only when color sensor detects red range 1000000 times a flag red_c is set to 1 
end
//freq_red=freq_red+1;          // once red_c==1, message sending happens and simultaneously color is also displayed on the led
if(sec_c==4999999 && (r_c>1000000))
begin
r_c=0;
g_c=0;
b_c=0;
red_c=1;
strt_delay=1;
end
end
///////////////////////////////////////////////////////////

else if(sec_c > 5000000 && sec_c<10000000 ) //Logic applied to Snippet 2 followed here for blue color detection
begin
s2=0;
s3=1;
if(f_count>16000 && f_count<18500 && (((c_node==34||c_node==29) &&c_d_n==1) ||((c_node==32||c_node==7)&&c_d_v==1)||((c_node==26||c_node==24||c_node==22)&&c_d_m==1)||((c_node==5||c_node==2)&&c_d_p==1)) && detect==0 )
begin
b_c=b_c+1;
end
//freq_blue=freq_blue+1;
if(sec_c==9999999 && (b_c>200000))
begin
b_c=0;
r_c=0;
g_c=0;
blue_c=1;
strt_delay=1;
end
end
/////////////////////////////////////////////////////////////////

else if(sec_c > 10000000 && sec_c<15000000)//Logic applied to Snippet 2 followed here for green color detection
begin
s2=1;
s3=1;
if( (((c_node==34||c_node==29)&&c_d_n==1) || ((c_node==32||c_node==7)&&c_d_v==1) ||((c_node==26||c_node==24||c_node==22)&&c_d_m==1)||((c_node==5||c_node==2)&&c_d_p==1)) && detect==0  && f_count>19000 && f_count<21000 )
begin
g_c=g_c+1;
end
//freq_green=freq_green+1;
if(sec_c==14999999 && (g_c>130000))
begin
g_c=0;
r_c=0;
b_c=0;
green_c=1;
strt_delay=1;
end
end
///////////////////////////////////////////////////////////////////
end


if (count1 == 10)
begin
count1 = 0;
flag1 = ~flag1;
end
if (count1<10)
begin
count1 = count1 +1;
end
end

assign adc_sck = flag1;


////////////////////////////////////////////////TASK TO TURN RIGHT,LEFT AND TAKE U TURNS///////////////////////////// 
task turn_left();
begin               //turns one wheel clockwise and the other anticlockwise to turn left
t_A_1a = 0;  
t_A_1b = 0; 
t_B_1a = 1; 
t_B_1b = 1;
end
endtask
 
task turn_right();
begin              //turns one wheel clockwise and the other anticlockwise to turn right
t_A_1a = 1;  
t_A_1b = 1;  
t_B_1a = 0; 
t_B_1b = 0;
end
endtask
 
task go_straight();
begin
t_A_1a = 1;      
t_A_1b = 0;  
t_B_1a = 1; 
t_B_1b = 0;
end
endtask

task u_turn();   //turns one wheel clockwise and the other anticlockwise to take u turn
begin
t_A_1a = 1;  
t_A_1b = 1;  
t_B_1a = 0; 
t_B_1b = 0;
end
endtask

////////////////////////////////////////////////////////DIJKSTRAS ALGORITHM////////////////////////////////////////////


always @(posedge clk_50)
begin
if(count_dj==0)
begin	
//////////////////////////////
for(a_dj=0;a_dj<36;a_dj=a_dj+1)   //a_dj and b_dj refer to row and column of the djkstras graph respectively
begin
	for(b_dj=0;b_dj<36;b_dj=b_dj+1) // The distances to various nodes are scaled and then stored as integers 
	begin
		dj_e[37*a_dj+b_dj]=0;        
		dj_d[37*a_dj+b_dj]=0;
		if((a_dj==0&&b_dj==19)||(a_dj==19&&b_dj==0)||(a_dj==2&&b_dj==3)||(a_dj==3&&b_dj==2)||(a_dj==5&&b_dj==6)||(a_dj==6&&b_dj==5)||(a_dj==7&&b_dj==8)||(a_dj==8&&b_dj==7)||(a_dj==5&&b_dj==9)||(a_dj==9&&b_dj==5)||(a_dj==9&&b_dj==10)||(a_dj==10&&b_dj==9)||(a_dj==10&&b_dj==11)||(a_dj==11&&b_dj==10)||(a_dj==10&&b_dj==13)||(a_dj==13&&b_dj==10)||(a_dj==12&&b_dj==13)||(a_dj==13&&b_dj==12)||(a_dj==13&&b_dj==14)||(a_dj==14&&b_dj==13)||(a_dj==13&&b_dj==16)||(a_dj==16&&b_dj==13)||(a_dj==15&&b_dj==16)||(a_dj==16&&b_dj==15)||(a_dj==16&&b_dj==17)||(a_dj==17&&b_dj==16)||(a_dj==16&&b_dj==19)||(a_dj==19&&b_dj==16)||(a_dj==18&&b_dj==19)||(a_dj==19&&b_dj==18)||(a_dj==19&&b_dj==20)||(a_dj==20&&b_dj==19)||(a_dj==22&&b_dj==23)||(a_dj==23&&b_dj==22)||(a_dj==24&&b_dj==25)||(a_dj==25&&b_dj==24)||(a_dj==26&&b_dj==27)||(a_dj==27&&b_dj==26)||(a_dj==28&&b_dj==29)||(a_dj==29&&b_dj==28)||(a_dj==29&&b_dj==30)||(a_dj==30&&b_dj==29)||(a_dj==32&&b_dj==33)||(a_dj==33&&b_dj==32)||(a_dj==34&&b_dj==35)||(a_dj==35&&b_dj==34))
		begin
			dj_e[37*a_dj+b_dj]=1;
			dj_d[37*a_dj+b_dj]=1;
		end
	end
end

// The following row and column for djkstras logic is determined by the formula (37*row + column) 
//and hence the graph is set as shown below  
	
dj_e[1]=1;
dj_d[1]=8;
dj_e[37]=1;
dj_d[37]=8;
dj_e[21]=1;
dj_d[21]=3;
dj_e[777]=1;
dj_d[777]=3;
dj_e[39]=1;
dj_d[39]=6;
dj_e[75]=1;
dj_d[75]=6;
dj_e[42]=1;
dj_d[42]=3;
dj_e[186]=1;
dj_d[186]=3;
dj_e[78]=1;
dj_d[78]=5;
dj_e[150]=1;
dj_d[150]=5;
dj_e[155]=1;
dj_d[155]=4;
dj_e[263]=1;
dj_d[263]=4;
dj_e[184]=0;
dj_d[184]=6;
dj_d[1336]=6;
dj_e[268]=1;
dj_d[268]=3;
dj_e[340]=1;
dj_d[340]=3;
dj_e[365]=1;
dj_d[365]=5;
dj_e[1193]=1;
dj_d[1193]=5;
dj_e[799]=1;
dj_d[799]=6;
dj_e[835]=1;
dj_d[835]=6;
dj_e[801]=1;
dj_d[801]=2;
dj_e[909]=1;
dj_d[909]=2;
dj_e[846]=1;
dj_d[846]=5;
dj_e[1206]=1;
dj_d[1206]=5;
dj_e[914]=1;
dj_d[914]=7;
dj_e[986]=1;
dj_d[986]=7;
dj_e[990]=1;
dj_d[990]=4;
dj_e[1062]=1;
dj_d[1062]=4;
dj_e[1072]=1;
dj_d[1072]=8;
dj_e[1360]=1;
dj_d[1360]=8;
dj_e[1104]=1;
dj_d[1104]=3;
dj_e[1176]=1;
dj_d[1176]=3;
dj_e[1179]=1;
dj_d[1179]=2;
dj_e[1215]=1;
dj_d[1215]=2;
dj_e[1181]=1;
dj_d[1181]=5;
dj_d[1289]=5;
dj_d[1294]=3;
dj_e[1366]=1;
dj_d[1366]=3;
	if(detect_run_ng==1) //Certain paths have been blocked for nutty ground 
	                     //for the color sensors to be on the correct side of the path 
		begin
		dj_e[1336]=0;
		dj_e[184]=0;
		dj_e[1294]=0;
		dj_e[1289]=1;
		end
	else if(detect_run_v==1)//Certain paths have been blocked for nutty ground 
	                        //for the color sensors to be on the correct side of the path 
		begin
		dj_e[1336]=1;
		dj_e[1294]=1;
		dj_e[1289]=0;
		end
	else if(detect_run_m==1)//Certain paths have been blocked for nutty ground 
	                        //for the color sensors to be on the correct side of the path 
		begin
		dj_e[1289]=1;
		dj_e[184]=1;
		dj_e[1062]=0;
		end
	else if(detect_run_p==1)//Certain paths have been blocked for nutty ground 
	                        //for the color sensors to be on the correct side of the path 
		begin
		dj_e[1062]=1;
		dj_e[155]=0;
		end
//////////////////////////////

k_dj=0;	
for(i_dj=0;i_dj<=36;i_dj=i_dj+1) //Each time a source and target is called the arrays
                                 //visited, store and store_final is reintialized
										   // as mentioned earlier store_final shows the bot the node number to be followed	
	begin
	visited[i_dj]=0;
	store[i_dj]=45;
	store_final[i_dj]=45;
	end

for(i_dj=0;i_dj<=36;i_dj=i_dj+1) // initially the distance array is assumed to be infinity (200) 
	begin
	distance[i_dj]=200;
	end
distance[s]=0; //distance from source to source is set as 0
end

if (count_u_dj<38 )
begin
if(count_dj==1)
	begin
	k_dj=0;
	nextd=200;///
	end
else if(count_dj>=2 && count_dj<=38)	
	nextvlist[count_dj-2]=37;         //intializing nextvlist with 37
	
else if(count_dj>=39 && count_dj<=75)	
		begin
		if(nextd>distance[count_dj-39] && visited[count_dj-39]==0) //the condition checks which among the connected nodes have the shortest distance
			begin
			nextd=distance[count_dj-39];
			end
		end
else if(count_dj>=76 && count_dj<=112)	// each time a different state machine is called for djikstras		
		begin
		if(visited[count_dj-76]==0 && distance[count_dj-76]==nextd)
			begin
			nextvlist[k_dj]=count_dj-76; //updates nextvlist if there are multiple nodes with the same distance from the node under consideration
			k_dj=k_dj+1;//make it a list
			end
		end
else if(count_dj==114)// 
		nextv=nextvlist[0];
		
else if(count_dj>=115 && count_dj<=150)
		//for(j=1;j<=36;j=j+1)//less than or equal to k
			begin
			if (nextv>nextvlist[count_dj-114])
					begin
					nextv=nextvlist[count_dj-114];
					end
			end
else if(count_dj==151 )//&& flag_empty==0)			
		visited[nextv]=1;
		
else if(count_dj>=152 && count_dj<=188)// && flag_empty==0)		
			begin
			
			if (dj_e[37*(count_dj-152)+nextv]==1 && visited[count_dj-152]==0)
				begin
				if (distance[count_dj-152]>=(distance[nextv]+dj_d[37*(count_dj-152)+nextv]))//
					begin
					store[count_dj-152]=nextv; //store array gets updated if the distance from source to that target is minimum
					distance[count_dj-152]=distance[nextv]+dj_d[37*(count_dj-152)+nextv]; //similiarly the distance array is also updated
					
					end
				
				end
				
			end//
else if (count_u_dj<38 && count_dj>188)
begin
count_u_dj=count_u_dj+1;             //ensures distance from source to all other nodes have been covered
count_dj=0;
end
if(count_u_dj==38)
begin                                //once all nodes have been covered the flags are reintitialized
flag_dj=1;
count_dj=count_dj-1;
end
end

///////////////////////
	count_dj=count_dj+1;
	if(flag_dj==1)
	begin
	t=target;
	store_final[0]=target;
////////////////////////////////////////////////////////TO OBTAIN THE PATH TO BE FOLLOWED IN REVERSE ORDER//////////////////////////
	for(i_dj=1;i_dj<37 && t!=s;i_dj=i_dj+1)
	begin
	store_final[i_dj]=store[t];
	t=store[t];
	end
////////////////////////////////////////////////////////TURNING THE REVERSED PATH TO ORIGINAL PATH ORDER//////////////////////////////////////
	for(z=0;z<37;z=z+1)
	begin
	if(store_final[z]==45 && flag_ven==0)
	begin
	ven=z;
	flag_ven=1;
	end
	end
	for(z=0;z<37;z=z+1)
	A[z]=store_final[ven-1-z];
	
	flag_dj=2;
	end
/////////////////////////////////////GAMECHANGER IF TURNED TO 1 TRIGGERS DIJKSTRAS ALGORITHM TO FIND THE MINIMUM DISTANCE PATH BETWEEN S AND TARGET//////////////	

	if(gamechanger==1 )//once all the following variables have been reinitialized djikstras algorithm gets set again
	begin
	count_u_dj=0;
	count_dj=0;
	flag_dj=0;
	flag_ven=0;
	k=0;
	gamechanger=0;
	for(z=0;z<37;z=z+1)
	A[z]=45;
	end
if(flag_dj==2)
begin




if(flagnode==0) // as a first step towrards bot run, the previous node c_node and n_node are set
begin
p_node=1;
c_node=0;
n_node=A[1];
flagnode=1;
k=1;
end



if(detect == 1) //once a node is reached the bot takes left right or goes straight depending upon the node numbers stored in A Matrix (store_final reversed) 
begin           //flags to turn right and left are set depending upon then next node previous node and current node
if((p_node==0 && c_node==1 && n_node==5)||(p_node==5 && c_node==1 && n_node==2)||(p_node==1 && c_node==2 && n_node==3)||(p_node==3 && c_node==2 && n_node==4)||(p_node==2 && c_node==4 && n_node==7)||(p_node==7 && c_node==4 && n_node==36)||(p_node==6 && c_node==5 && n_node==1)||(p_node==9 && c_node==5 && n_node==6)||(p_node==9 && c_node==7 && n_node==8)||(p_node==8 && c_node==7 && n_node==4)||(p_node==10 && c_node==9 && n_node==32)||(p_node==5 && c_node==9 && n_node==10)||(p_node==7 && c_node==9 && n_node==5)||(p_node==32 && c_node==9 && n_node==7)||(p_node==13 && c_node==10 && n_node==11)||(p_node==11 && c_node==10 && n_node==9)||(p_node==10 && c_node==13 && n_node==12)||(p_node==12 && c_node==13 && n_node==16)||(p_node==16 && c_node==13 && n_node==14)||(p_node==14 && c_node==13 && n_node==10)||(p_node==19 && c_node==16 && n_node==17)||(p_node==15 && c_node==16 && n_node==19)||(p_node==13 && c_node==16 && n_node==15)||(p_node==17 && c_node==16 && n_node==13)||(p_node==16 && c_node==19 && n_node==18)||(p_node==18 && c_node==19 && n_node==0)||(p_node==20 && c_node==19 && n_node==16)||(p_node==0 && c_node==19 && n_node==20)||(p_node==22 && c_node==21 && n_node==0)||(p_node==23 && c_node==22 && n_node==32)||(p_node==21 && c_node==22 && n_node==23)||(p_node==25 && c_node==24 && n_node==21)||(p_node==26 && c_node==24 && n_node==25)||(p_node==24 && c_node==21 && n_node==22)||(p_node==27 && c_node==26 && n_node==24)||(p_node==28 && c_node==26 && n_node==27)||(p_node==29 && c_node==28 && n_node==26)||(p_node==36 && c_node==28 && n_node==29)||(p_node==28 && c_node==29 && n_node==30)||(p_node==30 && c_node==29 && n_node==31)||(p_node==29 && c_node==31 && n_node==34)||(p_node==34 && c_node==31 && n_node==32)||(p_node==31 && c_node==34 && n_node==35)||(p_node==35 && c_node==34 && n_node==36)||(p_node==34 && c_node==36 && n_node==28)||(p_node==4 && c_node==36 && n_node==34)||(p_node==21 && c_node==0 && n_node==19)||(p_node==19 && c_node==0 && n_node==1)||(p_node==22 && c_node==32 && n_node==31)||(p_node==9 && c_node==32 && n_node==22)||(p_node==33 && c_node==32 && n_node==9)||(p_node==31 && c_node==32 && n_node==33))
flagturnleft=1;
else if((p_node==5 && c_node==1 && n_node==0)||(p_node==2 && c_node==1 && n_node==5)||(p_node==3 && c_node==2 && n_node==1)||(p_node==4 && c_node==2 && n_node==3)||(p_node==36 && c_node==4 && n_node==7)||(p_node==7 && c_node==4 && n_node==2)||(p_node==1 && c_node==5 && n_node==6)||(p_node==6 && c_node==5 && n_node==9)||(p_node==8 && c_node==7 && n_node==9)||(p_node==4 && c_node==7 && n_node==8)||(p_node==10 && c_node==9 && n_node==5)||(p_node==5 && c_node==9 && n_node==7)||(p_node==7 && c_node==9 && n_node==32)||(p_node==32 && c_node==9 && n_node==10)||(p_node==9 && c_node==10 && n_node==11)||(p_node==11 && c_node==10 && n_node==13)||(p_node==10 && c_node==13 && n_node==14)||(p_node==12 && c_node==13 && n_node==10)||(p_node==16 && c_node==13 && n_node==12)||(p_node==14 && c_node==13 && n_node==16)||(p_node==19 && c_node==16 && n_node==15)||(p_node==15 && c_node==16 && n_node==13)||(p_node==13 && c_node==16 && n_node==17)||(p_node==17 && c_node==16 && n_node==19)||(p_node==16 && c_node==19 && n_node==20)||(p_node==18 && c_node==19 && n_node==16)||(p_node==20 && c_node==19 && n_node==0)||(p_node==0 && c_node==19 && n_node==18)||(p_node==32 && c_node==22 && n_node==23)||(p_node==23 && c_node==22 && n_node==21)||(p_node==21 && c_node==24 && n_node==25)||(p_node==25 && c_node==24 && n_node==26)||(p_node==0 && c_node==21 && n_node==22)||(p_node==22 && c_node==21 && n_node==24)||(p_node==24 && c_node==26 && n_node==27)||(p_node==27 && c_node==26 && n_node==28)||(p_node==26 && c_node==28 && n_node==29)||(p_node==29 && c_node==28 && n_node==36)||(p_node==31 && c_node==29 && n_node==30)||(p_node==30 && c_node==29 && n_node==28)||(p_node==34 && c_node==31 && n_node==29)||(p_node==32 && c_node==31 && n_node==34)||(p_node==22 && c_node==32 && n_node==9)||(p_node==9 && c_node==32 && n_node==33)||(p_node==33 && c_node==32 && n_node==31)||(p_node==31 && c_node==32 && n_node==22)||(p_node==36 && c_node==34 && n_node==35)||(p_node==35 && c_node==34 && n_node==31)||(p_node==34 && c_node==36 && n_node==4)||(p_node==28 && c_node==36 && n_node==34)||(p_node==1 && c_node==0 && n_node==19)||(p_node==19 && c_node==0 && n_node==21))
flagturnright=1;
else if((p_node==0 && c_node==1 && n_node==2)||(p_node==2 && c_node==1 && n_node==0)||(p_node==1 && c_node==2 && n_node==4)||(p_node==4 && c_node==2 && n_node==1)||(p_node==36 && c_node==4 && n_node==2)||(p_node==2 && c_node==4 && n_node==36)||(p_node==1 && c_node==5 && n_node==9)||(p_node==21 && c_node==22 && n_node==32)||(p_node==32 && c_node==22 && n_node==21)||(p_node==21 && c_node==24 && n_node==26)||(p_node==26 && c_node==24 && n_node==21)||(p_node==9 && c_node==5 && n_node==1)||(p_node==9 && c_node==7 && n_node==4)||(p_node==4 && c_node==7 && n_node==9)||(p_node==10 && c_node==9 && n_node==7)||(p_node==5 && c_node==9 && n_node==32)||(p_node==7 && c_node==9 && n_node==10)||(p_node==32 && c_node==9 && n_node==5)||(p_node==13 && c_node==10 && n_node==9)||(p_node==9 && c_node==10 && n_node==13)||(p_node==10 && c_node==13 && n_node==16)||(p_node==12 && c_node==13 && n_node==14)||(p_node==16 && c_node==13 && n_node==10)||(p_node==14 && c_node==13 && n_node==12)||(p_node==19 && c_node==16 && n_node==13)||(p_node==15 && c_node==16 && n_node==17)||(p_node==13 && c_node==16 && n_node==19)||(p_node==17 && c_node==16 && n_node==15)||(p_node==16 && c_node==19 && n_node==0)||(p_node==18 && c_node==19 && n_node==20)||(p_node==20 && c_node==19 && n_node==18)||(p_node==0 && c_node==19 && n_node==16)||(p_node==0 && c_node==21 && n_node==24)||(p_node==24 && c_node==21 && n_node==0) ||(p_node==24 && c_node==26 && n_node==28)||(p_node==28 && c_node==26 && n_node==24)||(p_node==26 && c_node==28 && n_node==36)||(p_node==36 && c_node==28 && n_node==26)||(p_node==28 && c_node==29 && n_node==31)||(p_node==31 && c_node==29 && n_node==28)||(p_node==29 && c_node==31 && n_node==32)||(p_node==32 && c_node==31 && n_node==29)||(p_node==22 && c_node==32 && n_node==33)||(p_node==9 && c_node==32 && n_node==31)||(p_node==33 && c_node==32 && n_node==22)||(p_node==31 && c_node==32 && n_node==9)||(p_node==31 && c_node==34 && n_node==36)||(p_node==36 && c_node==34 && n_node==31)||(p_node==28 && c_node==36 && n_node==4)||(p_node==4 && c_node==36 && n_node==28)||(p_node==21 && c_node==0 && n_node==1)||(p_node==1 && c_node==0 && n_node==21)) 
go_straight();
else if((c_node==3)||(c_node==6)||(c_node==8)||(c_node==11)||(c_node==12)||(c_node==14)||(c_node==15)||(c_node==17)||(c_node==18)||(c_node==20)||(c_node==23)||(c_node==25)||(c_node==27)||(c_node==30)||(c_node==33)||(c_node==35))
flaguturn=1;
end


/////////////////////////////////////////////DELAY FOR THE NEXT NODE TO BE DETECTED///////////////////////////////////////

if(color_timer < 160000000 && (g_stop==1||b_stop==1||r_stop==1)) //color timer resets flags to zero for the color sensor to detect same colors 
																					  //in the same ground at different positions
color_timer=color_timer+1;                                       //
if(color_timer==160000000)
begin
color_timer=0;
g_stop=0;
b_stop=0;
r_stop=0;
end

if(timer<20000000 && detect==1) //timer is used to put up a delay for node detection
timer=timer+1;
if(timer==20000000)
begin
timer=0;
detect=0;
end

if(sec<48000000 && (flagturnleft==1 || flagturnright==1)) //The following timer calls turn left or turn right task and 
                                                          //when breaker==1 the line sensor starts to function and gets turned on after turning for a few clock cycles
																			 //This makes the bot turn any angle let it be 90 or obtuse 
sec=sec+1;
if(sec==19299998)
sec=19499999;

if(sec>19500000 && sec<48000000 && (flagturnleft==1 || flagturnright==1))
begin
if(sec==40000000)
begin
breaker=1;
detect=0;
end
if(sec==44000000)
begin
sec=44999999;
end
if(flagturnleft==1)
turn_left();
if(flagturnright==1)
turn_right();
end
if(sec==48000000 && (flagturnleft==1 || flagturnright==1)) //once turning is done all variables are reinitialized
begin
breaker=0;
flagturnleft=0;
flagturnright=0;
if(c_node==19 || c_node==34)
flagleft=1;
sec=0;

//
//

end

if(c_node==4 && drop_spot[0]==0)                 //to stop the bot if all requirements has been satisfied
begin
t_A_1a = 0;  
t_A_1b = 0; 
t_B_1a = 0; 
t_B_1b = 0;
end

if(secu<85000000 && flaguturn==1)                // A similiar logic is implemented for u turn
secu=secu+1;
if(secu>10000000 && secu<20000000)               // first the bot goes straight
begin
t_A_1a = 1;  
t_A_1b = 0; 
t_B_1a = 1; 
t_B_1b = 0;
end
if(secu>20000000 && secu<30000000)               // the bot stays at that position for a few clock pulses
begin
t_A_1a = 0;  
t_A_1b = 0; 
t_B_1a = 0; 
t_B_1b = 0;
if((c_node==6 && drop_spot[1]==0)||(c_node==3 ))
secu=85000000;                    //  
end
if(secu>30000000 && secu<85000000)
begin
if(secu==61000000)                               // breaker is set to 1 to detect line
breaker=1;
if(flaguturn==1)
u_turn();                                        // takes a u turn once the bot stays in position fora few clock pulses
end 
if(secu==85000000 && flaguturn==1)               // all values reset
begin
breaker=0;
flaguturn=0;
secu=0;
//
detect=0;
//
end
////////////////////////////////////////////FEEDING SOURCE AND TARGET INTO DIJKSTRAS LOGIC/////////////////////////////

if(detect_run_ng==1||detect_run_v==1||detect_run_m==1||detect_run_p==1) // Main logic for updating deposition node and detecting color
																								// when the bot is either in ng or v or m or p (name sof different fields) the condition gets satisfied
begin
if(green_c==1 && g_stop==0 )                                            //once g_stop=1 green deposition feeding doesnt occur multiple times in consecutive clock pulses 
begin
g_stop=1;
drop_spot_c[d_k]=c_node+1;                                              //each time the code gets in the condition, d_k iterates and drop_spot is updated
drop_spot[d_k]=c_node+1;                                                // similiarly a copy of drop_spot which is used to switch off leds when the deposition position is reached
drop_color[d_k]=2;                                                      // setting drop_color to 2 implies green is the requirement for the deposition spot that was detected
d_k=d_k+1;
end
if(blue_c==1 && b_stop==0 )                                             //similiar to case reffered above
begin
b_stop=1;
drop_spot_c[d_k]=c_node+1;
drop_spot[d_k]=c_node+1;
drop_color[d_k]=3;
d_k=d_k+1;
end
if(red_c==1 && r_stop==0 )                                              //similiar to case reffered above
begin
r_stop=1;
drop_spot_c[d_k]=c_node+1;
drop_spot[d_k]=c_node+1;
drop_color[d_k]=1;
d_k=d_k+1;
end
end




if((d_out_ch6>3550 && d_out_ch5>3450)||(d_out_ch7>3350 && d_out_ch5>3450)) //condition for node detection
begin
if(detect==0)                                                              //if detect node flag is zero then detect is first turned 1 
begin                                													//next node, previous node and current nodes are updated
detect=1;
p_node=c_node;
c_node=n_node;
k=k+1;
n_node=A[k];
///////////////////////////////////////////////////////////////////////
if(detect_run_ng==1)                                                       // indicates that the bot is currently running in nutty ground
begin

if(n_node==36)                                                             // if the target is reached the next target is set 
																									// so that the bot traverses through all the color patches in nutty ground
begin
s=n_node;
target=31;
gamechanger=1;
end

if(c_node==31&& p_node!=32)                                               // once c_node turns 31, the bot would have detected all color patches in that field
begin

                                                                          // NOTE: The following Snippet 3 is used for the bot to determine the shortest path to the color that needs to be picked  
                                                                          // This code has been used in various parts and will be reffered now on as Snippet 3
/////////////////////////////////////////////////////SNIPPET 3///////////////////////////////////////

c_d_n=0;																			// since color detection in nutty groung is over c_d_n is set to 0                                                                
if(drop_color[0]!=0||drop_color[1]!=0||drop_color[2]!=0)       // now the bot checks whether there are any depositions left. If there are any, the control flow enters the following condition 
begin

if(drop_color[0]!=0 && drop_spot_0==0)									// if drop_spot_0 is not yet attended the bot sets source and target for supply run
begin
s=32;
n_node=32;
if(drop_color[0]==1)                                           // and if the SI requirement for that node is pesticide, the target is set to the shortest red block
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[0]==2)															// and if the SI requirement for that node is nutrients, the target is set to the shortest green block
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[0]==3)															// and if the SI requirement for that node is water, the target is set to the shortest blue block
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_0=1;
gamechanger=1;
end

else if(drop_color[1]!=0 && drop_spot_1==0)                   // IN case drop_spot[0] is already fulfilled and now drop_spot[1] is left, a procedure similiar to that for drop_spot[1] is followed
begin
s=32;
n_node=32;
if(drop_color[1]==1)                                          // similiar to drop_spot[0]
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[1]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[1]==3)															// similiar to drop_spot[0]
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_1=1;
gamechanger=1;
end

else if(drop_color[2]!=0 && drop_spot_2==0)							//At last if there is any requirement by a third patch in the field, the bot sets target and source for that run
																					//logic followed similiar to that of drop_spot[0]
begin
s=32;
n_node=32;
if(drop_color[2]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[2]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[2]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_2=1;
gamechanger=1;
end
end

else
begin              //if all requirements in nutty ground field has been met the bot prepares to move on the next field
s=32;
n_node=32;
target=9;          //source and targets are set
gamechanger=1;
detect_run_ng=0;   // the run for nutty ground ends and run for maize ground begins
detect_run_v=1;
d_k=0;
g_stop=0;         // enables the color sensors to detect values again
b_stop=0;         
r_stop=0;
drop_spot_0=0;    // Since all requirements are now satisfied for nutty ground, all drop_spots and drop_colors are cleared 
drop_spot_1=0;
drop_spot_2=0;
drop_spot_c[0]=0;
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
drop_color[0]=0;
drop_color[1]=0;
drop_color[2]=0;
c_d_v=1;         // Bot now sets to run for vegetable ground
end

end
/////////////////////////////////////////////////SNIPPET 3 ENDS///////////////////////////////
end



////////////////////////////////////////////////// Detect Run for Vegetable Ground
if(detect_run_v==1)
begin
if(n_node==9 && visited_9==0) // once node 9 is reached the bot turns to run for requirements in vegetable ground
begin
drop_spot_c[0]=0;             
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
visited_9=1;
s=n_node;
target=36;
gamechanger=1;
end
if(c_d_v==0 && n_node==9 && (p_node==33||p_node==8))      // NOTE please refer Snippet 3 for the logic below
begin
c_d_v=0;
if(drop_color[0]!=0||drop_color[1]!=0||drop_color[2]!=0)  // Logic in Snippet 3
begin

if(drop_color[0]!=0 && drop_spot_0==0)                   // Logic in Snippet 3
begin
s=n_node;
if(drop_color[0]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[0]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[0]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_0=1;
gamechanger=1;
end

else if(drop_color[1]!=0 && drop_spot_1==0)             // Logic in Snippet 3
begin
s=n_node;
if(drop_color[1]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[1]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[1]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_1=1;
gamechanger=1;
end

else if(drop_color[2]!=0 && drop_spot_2==0)         // Logic in Snippet 3
begin
s=n_node;
if(drop_color[2]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[2]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[2]==3)                              // Logic in Snippet 3
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_2=1;
gamechanger=1;
end
end

else                                              // Logic in Snippet 3
begin
s=n_node;
target=26;
gamechanger=1;
detect_run_ng=0;
detect_run_v=0;
detect_run_m=1;
d_k=0;
g_stop=0;
b_stop=0;
r_stop=0;
drop_spot_0=0;
drop_spot_1=0;
drop_spot_2=0;
drop_spot_c[0]=0;                                   // Logic in Snippet 3
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
drop_color[0]=0;
drop_color[1]=0;
drop_color[2]=0;
c_d_m=1;
end

end

if(n_node==36)    // Since there might be multiple positions in which the bot might end the run, decision making is taken multiple points  
begin
////////////////////////////////Logic IN SNIPPET 3///////////////////////////
c_d_v=0;          
if(drop_color[0]!=0||drop_color[1]!=0||drop_color[2]!=0)		//Logic IN SNIPPET 3
begin

if(drop_color[0]!=0 && drop_spot_0==0)
begin
s=n_node;
if(drop_color[0]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[0]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[0]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_0=1;
gamechanger=1;
end

else if(drop_color[1]!=0 && drop_spot_1==0)					//Logic IN SNIPPET 3
begin
s=n_node;
if(drop_color[1]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[1]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[1]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_1=1;
gamechanger=1;
end

else if(drop_color[2]!=0 && drop_spot_2==0)					//Logic IN SNIPPET 3
begin
s=n_node;
if(drop_color[2]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[2]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[2]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_2=1;
gamechanger=1;
end
end

else																	//Logic IN SNIPPET 3
begin
s=n_node;
target=26;
gamechanger=1;
detect_run_ng=0;
detect_run_v=0;
detect_run_m=1;
d_k=0;
g_stop=0;
b_stop=0;
r_stop=0;
drop_spot_0=0;
drop_spot_1=0;
drop_spot_2=0;

drop_spot_c[0]=0;											//Logic IN SNIPPET 3
drop_spot_c[1]=0;
drop_spot_c[2]=0;

drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
drop_color[0]=0;
drop_color[1]=0;
drop_color[2]=0;										 //Logic IN SNIPPET 3
c_d_m=1;
end

end

end
///////////////////////////////////////////////////Maize Terrain Run//////////////

if(detect_run_m==1)              // Once the bot has moved through vegetable garden, maize terrain is now the run spot
begin
if(n_node==26 && c_d_m==1)       // when the bot reaches node 26, maize terrain has now reached and run continues similiar to other terrains
begin
drop_spot_c[0]=0;
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
s=n_node;
target=32;
gamechanger=1;
end
if(n_node==32 && p_node==21 && c_d_m==1)
begin
s=n_node;
target=9;
gamechanger=1;
end
//////////////////////////////////////////////////Logic IN SNIPPET 3////////////////////////
if(n_node==9&& p_node==22 && c_d_m==1)
begin
c_d_m=0;
if(drop_color[0]!=0||drop_color[1]!=0||drop_color[2]!=0)            //Logic IN SNIPPET 3
begin

if(drop_color[0]!=0 && drop_spot_0==0)                              
begin
s=n_node;
if(drop_color[0]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[0]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[0]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_0=1;
gamechanger=1;
end

else if(drop_color[1]!=0 && drop_spot_1==0)                         //Logic IN SNIPPET 3
begin
s=n_node;
if(drop_color[1]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[1]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[1]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_1=1;
gamechanger=1;
end

else if(drop_color[2]!=0 && drop_spot_2==0)                       //Logic IN SNIPPET 3
begin
s=n_node;
if(drop_color[2]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[2]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[2]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_2=1;
gamechanger=1;
end
end

else                                                               //Logic IN SNIPPET 3
begin
s=n_node;
target=1;
gamechanger=1;
detect_run_ng=0;
detect_run_v=0;
detect_run_m=0;
detect_run_p=1;
d_k=0;
g_stop=0;
b_stop=0;
r_stop=0;
drop_spot_0=0;
drop_spot_1=0;
drop_spot_2=0;
drop_spot_c[0]=0;
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;                                                       //Logic IN SNIPPET 3
drop_spot[1]=0;
drop_spot[2]=0;
drop_color[0]=0;
drop_color[1]=0;
drop_color[2]=0;
c_d_p=1;
end

end

end
/////////////////////////////////////////////////////////Run in Paddy Plane//////////////////////

if(detect_run_p==1)                       // Once the bot has moved through maize terrain, paddy plane is now the run spot
begin
if(n_node==9 && c_d_p==1)						// considering different paths through which the bot may run into paddy plane, the targets are set to run for color detection in paddy plane
begin
s=n_node;
target=1;
gamechanger=1;
end
if(n_node==1 && c_d_p==1)
begin
s=n_node;
target=4;
gamechanger=1;
end

if(n_node==4 && c_d_p==1)
begin
s=n_node;
target=9;
gamechanger=1;
end
/////////////////////////////////////////////////////////////Logic in Snippet 3/////////////////////////////
if(n_node==9&& (p_node==4||p_node==6))
begin
c_d_p=0;
if(drop_color[0]!=0||drop_color[1]!=0||drop_color[2]!=0) //Logic in Snippet 3
begin

if(drop_color[0]!=0 && drop_spot_0==0)
begin
s=n_node;
if(drop_color[0]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[0]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[0]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_0=1;
gamechanger=1;
end

else if(drop_color[1]!=0 && drop_spot_1==0)   //Logic in Snippet 3
begin
s=n_node;
if(drop_color[1]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[1]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[1]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_1=1;
gamechanger=1;
end

else if(drop_color[2]!=0 && drop_spot_2==0)    //Logic in Snippet 3
begin
s=n_node;
if(drop_color[2]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[2]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[2]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_2=1;
gamechanger=1;
end
end

else                                              //Logic in Snippet 3
begin
s=n_node;
target=0;
gamechanger=1;
detect_run_ng=0;
detect_run_v=0;
detect_run_m=0;
detect_run_p=0;
d_k=0;
g_stop=0;
b_stop=0;
r_stop=0;
drop_spot_0=0;
drop_spot_1=0;
drop_spot_2=0;
drop_spot_c[0]=0;
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
drop_color[0]=0;
drop_color[1]=0;
drop_color[2]=0;
c_d_p=0;                                         //Logic in Snippet 3
end


end


end


///////////////////If any of the dead end nodes are reached, that indicates that deposition point has reached

if(n_node==11||n_node==14||n_node==12||n_node==15||n_node==17||n_node==20||n_node==18)  //
begin
if (s==32||s==36||s==9||s==27||s==25||s==23) // if a deposition point is reached the drop_spot is set to 0
begin
if(drop_spot[0]!=0||drop_spot[1]!=0||drop_spot[2]!=0)
begin
if(drop_spot[0]!=0)
target=drop_spot[0];
else if(drop_spot[1]!=0)
target=drop_spot[1];
else
target=drop_spot[2];
end
end
s=n_node;
gamechanger=1;
end


if(n_node==drop_spot[0])            // to set drop spots to 0
begin
drop_spot[0]=0;
drop_color[0]=0;
end
if(n_node==drop_spot[1])
begin
drop_spot[1]=0;
drop_color[1]=0;
end
if(n_node==drop_spot[2])
begin
drop_spot[2]=0;
drop_color[2]=0;
end

                                      //If an end node is reached in nutty ground then it gets directed to node 31
if(n_node==35||n_node==30)
begin
s=n_node;
target=31;
gamechanger=1;
end

if(n_node==33||n_node==8)				//If an end node is reached in nutty ground then it gets directed to node 31
begin
s=n_node;
target=9;
gamechanger=1;
end

if(n_node==27||n_node==25||n_node==23)	//If an end node is reached in maize ground then according to snippet 3 logic, dijkstras algoritm searches for the shortest path towards the target required
begin
///////////////////////////////////////Logic in Snippet 3///////////////////
c_d_m=0;

if(drop_color[0]!=0||drop_color[1]!=0||drop_color[2]!=0)     //Logic in Snippet 3
begin

if(drop_color[0]!=0 && drop_spot_0==0)
begin
s=n_node;
if(drop_color[0]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
else if(drop_color[0]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
else if(drop_color[0]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_0=1;
gamechanger=1;
end

else if(drop_color[1]!=0 && drop_spot_1==0)           //Logic in Snippet 3
begin
s=n_node;
if(drop_color[1]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[1]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[1]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_1=1;
gamechanger=1;
end

else if(drop_color[2]!=0 && drop_spot_2==0)            //Logic in Snippet 3
begin
s=n_node;
if(drop_color[2]==1)
begin
target=redpick[r_k];
r_k=r_k+1;
end
if(drop_color[2]==2)
begin
target=greenpick[g_k];
g_k=g_k+1;
end
if(drop_color[2]==3)
begin
target=bluepick[b_k];
b_k=b_k+1;
end
drop_spot_2=1;
gamechanger=1;
end
end

else                                         //Logic in Snippet 3
begin
s=n_node;
target=9;
gamechanger=1;
detect_run_ng=0;
detect_run_v=0;
detect_run_m=0;
detect_run_p=1;
d_k=0;
g_stop=0;
b_stop=0;
r_stop=0;
drop_spot_0=0;
drop_spot_1=0;
drop_spot_2=0;
drop_spot_c[0]=0;
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
drop_color[0]=0;
drop_color[1]=0;
drop_color[2]=0;
c_d_p=1;
end


///////////////
end
if((c_node==23||c_node==25||c_node==27) && (drop_color[0]==0 && drop_color[1]==0 && drop_color[2]==0))   //Logic in Snippet 3
begin
s= c_node-1;
target=9;
gamechanger=1;
detect_run_ng=0;
detect_run_v=0;
detect_run_m=0;
detect_run_p=1;
d_k=0;
g_stop=0;
b_stop=0;
r_stop=0;
drop_spot_0=0;
drop_spot_1=0;
drop_spot_2=0;
drop_spot_c[0]=0;
drop_spot_c[1]=0;
drop_spot_c[2]=0;
drop_spot[0]=0;
drop_spot[1]=0;
drop_spot[2]=0;
drop_color[0]=0;
drop_color[1]=0;
drop_color[2]=0;
c_d_p=1;
end

if(n_node==3||n_node==6)        //If the bot ends in paddy field the bot stops hence no seperate code is required for djkstras algorithm at paddy plane  
begin
s=n_node;
target=9;
gamechanger=1;
end


end
end


//////////////////////////////////////////////////LINE FOLLOWER/////////////////////////////////////////////

else if(d_out_ch5>2850 && ((flagturnright==0 && flagturnleft==0 && flaguturn==0)||breaker==1)) // detcts line to go straight
begin
t_A_1a = 1;  
t_A_1b = 0; 
t_B_1a = 1; 
t_B_1b = 0;
end  

else if(d_out_ch7>900 && ((flagturnright==0 && flagturnleft==0 && flaguturn==0)||breaker==1))  // detcts line to go right
begin
t_A_1a = 1;  
t_A_1b = 0; 
t_B_1a = 0; 
t_B_1b = 0;
end 
else if(d_out_ch6>2500 && ((flagturnright==0 && flagturnleft==0 && flaguturn==0)||breaker==1))  // detcts line to go ledt
begin
t_A_1a = 0;  
t_A_1b = 0; 
t_B_1a = 1; 
t_B_1b = 0;
end

else if(flagturnright==0 && flagturnleft==0 && flaguturn==0) // stop if white is detected
begin
t_A_1a = 0;  
t_A_1b = 0; 
t_B_1a = 0; 
t_B_1b = 0;
end
else
nkmn=1;
if(flag_game==1)
gamechanger=0;
if(target==0 && c_node==0)
begin
t_A_1a = 0;  
t_A_1b = 0; 
t_B_1a = 0; 
t_B_1b = 0;
end
end
end
//////////////////////////////////////////////ADC control///////////////////////////////////
always @(posedge adc_sck)
begin
if (count2 == 1)
cs_n_2 = 0;
if (count2 == 16)
cs_n_2 = 1;
end

always @(negedge adc_sck)
begin
alt = 0;
if (count2 == 0)
begin
cs_n = 0;
start = 0;
end
if (count2 == 16)
begin
alt = 1;
if (flag2 == 2)
begin
  
flag2 = 0;
ctrl6 = 12'b111111111111;
  
end
else
begin
  
flag2 = flag2 +1;
if (flag2 == 1)
begin
ctrl7 = 12'b111111111111;
end
if (flag2 == 2)
begin
ctrl5 = 12'b111111111111;
end
end
count2 = 0;
end
if (count2<16)
count2 = count2 +1;
if(count2 == 6)
begin
t_din = 0;
a=11;
start = 1;
end
if (count2 == 3|| count2 == 4 || count2 == 5)
begin
if (data_frame == 0)
begin
if (count2 == 3)
t_din = 1;
if (count2 == 4)
t_din = 0;
if (count2 == 5)
begin
t_din = 1;
end
end
if (data_frame == 1)
begin
if (count2 == 3)
t_din = 1;
if (count2 == 4)
t_din = 1;
if (count2 == 5)
begin
t_din = 0;
end
end
if (data_frame == 2)
begin
if (count2 == 3)
t_din = 1;
if (count2 == 4)
t_din = 1;
if (count2 == 5)
begin
t_din = 1;
end
end
end 
if (start == 1)
begin
if (data_frame == 0 && a>=0)
begin
t_d_out_ch7[a] = dout;
a = a-1;
end
if (data_frame == 1 && a>=0)
begin
t_d_out_ch5[a] = dout;
a = a-1;
end
if (data_frame == 2 && a>=0)
begin
t_d_out_ch6[a] = dout;
a = a-1;
end
end
end
assign data_frame = flag2;

////////////////////////YOUR CODE ENDS HERE////////////////////////


endmodule
///////////////////////////////MODULE ENDS//////////////////////////