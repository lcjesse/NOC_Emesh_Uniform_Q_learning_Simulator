#ifndef electronic_mesh_router
#define electronic_mesh_router

#include "electronic_mesh_definition.h"
#include "global_data.h"
//IMPORTANT NOTICE:: YOU SHOULD NEVER CHANGE VARIABLE TO SIGNAL TYPE OR FROM SIGNAL TYPE TO NORMAL VARIABLE---UNLESS YOU ARE PRETTY SURE WHAT YOU ARE DOING
SC_MODULE(Router)
{
	sc_in<bool> clk;
	sc_in<router_forward_interface> input_forward_interface[port_no];
	sc_out<router_backward_interface> input_backward_interface[port_no];
	
	sc_out<router_forward_interface> output_forward_interface[port_no];
	sc_in<router_backward_interface> output_backward_interface[port_no];

//router_ID
	int id;
	int pos_x;
	int pos_y;
//Store the interface data at each rising edge of clk
	router_forward_interface input_forward_data[port_no];
	router_backward_interface input_backward_data[port_no];
	router_forward_interface output_forward_data[port_no];
	router_backward_interface output_backward_data[port_no];
//input 
	enum input_state_type{vc_arbitration, wait_grant, sending_data};
	sc_signal<input_state_type> input_port_state[port_no];

	enum output_state_type{input_arbitration, input_granted};
	sc_signal<output_state_type> output_port_state[port_no];

	flit_type buffer[port_no][vc_number][vc_depth]; //buffer is organized as VCs with each VC including some flits
	//int write_pointer[port_no][vc_number]; //read, write pointer for buffer
	vector< vector<int> > write_pointer;
	//int read_pointer[port_no][vc_number]; 
	vector< vector<int> > read_pointer;
	//int old_write_pointer[port_no][vc_number];//for checking buffer change
	vector< vector<int> > old_write_pointer;
	//int old_read_pointer[port_no][vc_number]; 
	vector< vector<int> > old_read_pointer; 
	//int vc_robin[port_no];// robin for vc of each input port
	vector<int> vc_robin; //
	//int input_robin[port_no];//robin for input port at each output port
	vector<int> input_robin;//
	//int granted_input[port_no];//granted input port at output port
	vector<int> granted_input;//
	//int current_output_direction[port_no];//aim output port of each input port
	vector<int> current_output_direction;// 
	//bool reset_ready_signal[port_no]; // for reset output data
	vector<bool> reset_ready_signal;//
	//int current_vc_id[port_no];// record current VC being processed at each input port
	vector< int > current_vc_id; //current_vc_id.resize(port_no,-1);

	vector< vector<int> > output_vc_reserve; // this is to record the states of vc in next router. So we can not send new packets to next vc if it is reserved by others
	                                          // if it is -1, means it is not reserved by any one

	struct vc_state_type{
		int vc_id;
		int input_id;
		int next_vc_id;
		int output_port_direction;
		bool empty;
		bool full;
		bool reserved;
		bool available_for_new;//this is similar to "reserved"; however, when tail flits come, the vc is avaible for new packet though it is storing a packet
	};
	vc_state_type vc_state[port_no][vc_number];//vc mapping table, mapping VC in this router to vc of following router
	vector< vector<int> >available_space;//[port_no][vc_number];

	sc_signal<bool> request_fail[port_no][port_no];//
	sc_signal<bool> request_success[port_no][port_no];//

	int previous_flit_number[port_no][vc_number];
	int stable_output_direction[port_no][vc_number];
	int stable_output_next_vc[port_no][vc_number];

//	ofstream router_log;


   void get_in_all_data();
   int routing_computation(flit_type head_flit);
   void input_port();
   void backward_of_input_port();
   void output_port();
   void initial();
   void setid(int num); 
   int getid(); 
   void setpos(int x, int y);
   void getpos(int &x, int &y);

   void get_xy(int address, int &x, int &y);

   int get_left();
   int get_right();
   int get_up();
   int get_down();
   int get_toproc();

   int get_waiting_time(double& receive_time, double& leave_time);///////////////////////////////////
   int(*qtable)[4] = new int[NOC_WIDTH * NOC_HEIGHT][4];
//int qtable[NOC_WIDTH * NOC_HEIGHT][4];///////////////////////////////Q table
   bool updated_latency_flag[port_no];/////////////////////////////////The update-q-value flag in inport after headflit request successfully
   bool check_if_arrived_dst[port_no];///////////////////////////////When certain of updating Q value, according to whether it has arrived dst to decide update as 'waiting_time' or 'Q + waiting_time'
   int updated_latency_dst[port_no];//////////////////////////////////////////////The signal to transmit dst
   double leave_time[port_no];////////////////////////////////////////////////////Router's head flit's leave time: When flit find available output port
   double receive_time[port_no];////////////////////////////////////////////////////Router's head flit's receive time: When head flit arrives. To calculate waiting time
 //  int vc_computation(flit_type head_flit, int input_port, int output_port, int old_vc);/////////////////////////////decide which vc to go

   SC_HAS_PROCESS(Router);
   Router(sc_module_name name=sc_gen_unique_name("Router"))
   {
		initial();
		

		SC_METHOD(get_in_all_data);
		sensitive<<clk.pos();

		SC_THREAD(input_port);
		sensitive<<clk.pos();

		SC_THREAD(backward_of_input_port);
		sensitive<<clk.pos();

		SC_THREAD(output_port);
		sensitive<<clk.pos();


   }

};

void Router::initial()
{
	//router id and its x and y positions
	id = -1;
	pos_x = -1;
	pos_y = -1;
	//initial output vc states, -1 means it is not reserved
	vector<int> temp_vc_reserve;
	for(int i=0;i<vc_number;i++)
		temp_vc_reserve.push_back(-1);
	for(int i=0;i<port_no;i++)
		output_vc_reserve.push_back(temp_vc_reserve);

	for (int i = 0; i < NOC_WIDTH * NOC_HEIGHT; i++)//////////////////////////////////////////////////////initial the qtable to zero
	{
		for (int j = 0; j < 4; j++)
		{
			qtable[i][j] = 0;
		}
	}

	vc_robin.resize(port_no, 0);
	input_robin.resize(port_no,0);
	granted_input.resize(port_no, -1);
	current_output_direction.resize(port_no, -1);
	reset_ready_signal.resize(port_no, false);
	current_vc_id.resize(port_no,-1);
	read_pointer.resize(port_no, vector<int>(vc_number,0));
	write_pointer.resize(port_no, vector<int>(vc_number,0));
	old_read_pointer.resize(port_no, vector<int>(vc_number,0));
	old_write_pointer.resize(port_no, vector<int>(vc_number,0));
	available_space.resize(port_no, vector<int>(vc_number, vc_depth));

	
	for(int i=0;i<port_no;i++)
	{
		for(int j=0;j<port_no;j++)
		{
			request_fail[i][j]=false;
			request_success[i][j]=false;

		}
	}

	for(int i=0;i<port_no;i++)
	{
		input_port_state[i]=vc_arbitration;
		output_port_state[i]=input_arbitration;

		updated_latency_flag[i] = false;/////////////////////////////////////////////////////////////////////initate Q value flag as false
		receive_time[i] = 0;/////////////////////////////////////////////////////////////////////////////////
		leave_time[i] = 0;//////////////////////////////////////////////////////////////////////////////////

		for(int j=0;j<vc_number;j++)
		{
			vc_state[i][j].available_for_new=true;
			vc_state[i][j].empty=true;
			vc_state[i][j].full=false;
			vc_state[i][j].input_id=i;
			vc_state[i][j].next_vc_id=-1;
			vc_state[i][j].reserved=false;
			vc_state[i][j].vc_id=j;

		}
	}
}

void Router::setid(int num) 
{
	id = num;
}
int Router::getid() 
{
	return id;
}
void Router::setpos(int x, int y) 
{
	pos_x = x;
	pos_y = y;
}
void Router::getpos(int &x, int &y) 
{
	x = pos_x; 
	y = pos_y;
}

void Router::get_xy(int address, int &x, int &y) 
{
	y = address / NOC_WIDTH;
	x = address % NOC_WIDTH;
	assert (address == y*NOC_WIDTH + x);
}

int Router::get_left() {
	return LEFT;
}
int Router::get_right() {
	return RIGHT;
}
int Router::get_up() {
	return UP;
}
int Router::get_down() {
	return DOWN;
}
int Router::get_toproc() {
	return TO_PROCESSOR;
}

void Router::get_in_all_data()//at rising edge of clk, get in data
{
	//cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] get in data"<<endl;

	for(int i=0;i<port_no;i++)
	{
		input_forward_data[i]=input_forward_interface[i].read();
		output_backward_data[i]=output_backward_interface[i].read();
		

		//here, we update routing delay 
		for(int j=0;j<vc_number;j++)
		{
			old_write_pointer.at(i).at(j)=write_pointer.at(i).at(j);

			for(int k=0;k<vc_depth;k++)
			{
				if(true == buffer[i][j][k].head && buffer[i][j][k].routing_delay>0)
				{
					buffer[i][j][k].routing_delay--;
				}
			}
		}
	}

	//then we store the data if needed
	for(int i=0;i<port_no;i++)
	{
		if(true == input_forward_data[i].ready)//if there is data on the port
		{
			int vc_id=input_forward_data[i].vc_id;
			/*router_log<<sc_simulation_time()<<" receives data input_id= "<<i<<" vc_id="<< vc_id <<"seq="<<input_forward_data[i].data_flit.packet_sequence
				<<"core "<<input_forward_data[i].data_flit.src<<"->"<<input_forward_data[i].data_flit.dest<<endl;
			*/
			//if it is a head flit, we need calculate the output port
			//and simulate routing delay
			if(true == input_forward_data[i].data_flit.head)
			{
				previous_flit_number[i][vc_id]=input_forward_data[i].data_flit.packet_sequence;

				receive_time[i] = double(sc_time_stamp().to_double() / 1000);//////////////////////////////////////////////////////record the head flit's receive time 
				//leave_time[i] = receive_time[i];//////////////////////////////////////////////////////////////////////////leave time for last router is equal to receive time 
				if (input_forward_data[i].data_flit.vc_choice != vc_id&&i!=TO_PROCESSOR)////////////////////////////////////////////////check if last vc_choice is correct
					cout << "vc_choice failed" << " " << input_forward_data[i].data_flit.vc_choice << " " << vc_id <<" "<<i<< endl;
				input_forward_data[i].data_flit.direction=routing_computation(input_forward_data[i].data_flit);
				stable_output_direction[i][vc_id]=input_forward_data[i].data_flit.direction;
				input_forward_data[i].data_flit.routing_delay=routing_algorithm_delay;
				//input_forward_data[i].data_flit.vc_choice = vc_computation(input_forward_data[i].data_flit, i, input_forward_data[i].data_flit.direction, vc_id);/////////////////

				if(false == vc_state[i][vc_id].available_for_new )
				{
					cout<<sc_simulation_time<<"error receiving head!!!";
				}
				if(true ==  vc_state[i][vc_id].reserved)
				{
					/*cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] receiving back-to-back packet"
						<< input_forward_data[i].data_flit.src<<"->"<< input_forward_data[i].data_flit.dest<<" id="
						<< input_forward_data[i].data_flit.packet_sequence<<endl;*/
				}

				vc_state[i][vc_id].reserved=true;
				vc_state[i][vc_id].available_for_new=false;

			}
			else//not head
			{
				if(1000000 != input_forward_data[i].data_flit.packet_sequence-previous_flit_number[i][vc_id])
				{
					cout<<sc_simulation_time()<<" error in nonconsistance"<<endl;
				}
				previous_flit_number[i][vc_id]=input_forward_data[i].data_flit.packet_sequence;

			}
			/*if(true == input_forward_data[i].data_flit.tail)
			{
				vc_state[i][vc_id].available_for_new=true;
			}*/

			if(input_forward_data[i].data_flit.tail==true)
			{
				//first of all, we calculate switching capacity consumed and number of routers the packet is encountered
				input_forward_data[i].data_flit.encountered_router_number++;
				//input_forward_data[i].data_flit.switching_capacity_consumed =flit_size * input_forward_data[i].data_flit.packet_length + input_forward_data[i].data_flit.switching_capacity_consumed;

				/////router power
				//input_forward_data[i].data_flit.power+=(XbarPower+BufferPower*1)*input_forward_data[i].data_flit.packet_length*32 + RouterControlPower;
				////wire power
			}

			buffer[i][vc_id][write_pointer.at(i).at(vc_id)]=input_forward_data[i].data_flit;
			write_pointer.at(i).at(vc_id)=(write_pointer.at(i).at(vc_id)+1)%vc_depth;

		}
	}


}

int Router::routing_computation(flit_type head_flit)
	{
	int dst = head_flit.dest;
	int src = head_flit.src;

	int dst_x = -1;
	int dst_y = -1;

	int src_x = -1;
	int src_y = -1;

	get_xy(dst, dst_x, dst_y);
	get_xy(src, src_x, src_y);


	int e_x = dst_x - pos_x;
	int e_y = dst_y - pos_y;

	vector<int> avil_d;

	int d, direction;
	//double temp_value;
	int temp_value_q;

	if (dst == head_flit.src) {
		return 1;
	}

	if ((dst_x == pos_x) && (dst_y == pos_y)) {
		return TO_PROCESSOR;
	}
	switch (routing_algo) {
	case 0://///////////////////////////////mesh q-learning minimum path
		if (pos_x != dst_x) {
			if (pos_x < dst_x) {
				avil_d.push_back(RIGHT);
			}
			else if (pos_x > dst_x) {
				avil_d.push_back(LEFT);
			}

		}
		if (pos_y != dst_y) {
			if (pos_y < dst_y) {
				avil_d.push_back(UP);
			}
			else if (pos_y > dst_y) {
				avil_d.push_back(DOWN);
			}

		}

		if (avil_d.size() == 2)////////////to avoid becoming dimension-orderd routing, we use avil[random] instead of avil[0] as a start. In this way, we can avoid choose left right first.
		{
			int d1 = avil_d[0];
			int d2 = avil_d[1];
			if (qtable[dst][d1] < qtable[dst][d2])
				d = d1;
			else if (qtable[dst][d1] > qtable[dst][d2])
				d = d2;
			else 
				d = avil_d[rand() % 2];
			//int i = rand() % avil_d.size();
			//d = avil_d[i];
			//temp_value_q = qtable[dst][d];
			//direction = avil_d[1 - i];
			//if (qtable[dst][direction] < temp_value_q)
			//{
			//	d = direction;
			//}
		}
		else if (avil_d.size() == 1)
			d = avil_d[0];

		//d = avil_d[0];
		//temp_value_q = qtable[dst][d];
		//for (int i = 1; i < avil_d.size(); i++) {
		//	direction = avil_d[i];

		//	if ((qtable[dst][direction]) < temp_value_q) {
		//		d = direction;
		//		temp_value_q = qtable[dst][d];
		//	}
		//}
		break;

	case 1:///////////////////////////////mesh xy dimension-orderd
		if (pos_x != dst_x) {
			if (pos_x < dst_x) {
				d = RIGHT;
			}
			else if (pos_x > dst_x) {
				d = LEFT;
			}
		}
		else if (pos_y != dst_y) {
			if (pos_y < dst_y) {
				d = UP;
			}
			else if (pos_y > dst_y) {
				d = DOWN;
			}
		}
		else {
			d = TO_PROCESSOR;
		}
		break;

	case 2://////////////////////////////////mesh odd-even
		if (e_x == 0 && e_y == 0) {
			return 0;
		}

		if (e_x == 0)
		{
			if (e_y > 0)
			{
				avil_d.push_back(2);														//x=0, y>0  UP
			}
			else {
				avil_d.push_back(3);														//x=0, y<0  DOWN
			}
		}
		else
		{
			if (e_x > 0) {
				if (e_y == 0) {
					avil_d.push_back(1);													//x>0, y=0  RIGHT
				}
				else
				{
					if (dst_x % 2 == 1 || e_x != 1) {
						avil_d.push_back(1);												//x>0, y!=0, x!=1 or dst is odd  RIGHT
					}
					if ((pos_x % 2 == 1) || pos_x == src_x) {
						if (e_y > 0) {
							avil_d.push_back(2);											//x>0, y>0, pos_x is odd or pos_x is src  UP
						}
						else {
							avil_d.push_back(3);											//x>0, y<0, pos_xΪis odd or pos_x is src  DOWN
						}
					}
				}
			}
			else {
				avil_d.push_back(0);														//x<0  LEFT
				if (pos_x % 2 == 0) {
					if (e_y > 0) {
						avil_d.push_back(2);												//x<0, pos_x is even, y>0  UP
					}
					else
					{
						avil_d.push_back(3);												//x<0, pos_x is even, y<=0  DOWN
					}
				}
			}
		}
		d = avil_d[rand() % avil_d.size()];
		break;

	case 3:///////////////////////////////////////mesh west-first
		if (dst_x == pos_x) {														//First check if it is in the same row, if true then go north or south
			if (dst_y > pos_y) {
				d = 2;
			}
			else {
				d = 3;
			}
		}
		else if (dst_y == pos_y) {												//then check if it is in the same column, if true then go east of west
			if (dst_x > pos_x) {
				d = 1;
			}
			else {
				d = 0;
			}
		}
		else if (dst_x < pos_x) {												//else if dst is in west side then go west
			d = 0;
		}
		else if (dst_y > pos_y) {												//else if dst is in north side then go north or east randomly
			avil_d.push_back(2);
			avil_d.push_back(1);

			d = avil_d[rand() % avil_d.size()];
		}
		else {																			//else if dst is in south side then go south or east randomly
			avil_d.push_back(1);
			avil_d.push_back(3);

			d = avil_d[rand() % avil_d.size()];
		}
		break;

	case 4://////////////////////////////////////////mesh negative first
		if (dst_x == pos_x) {
			if (dst_y > pos_y) {
				d = 2;
			}
			else {
				d = 3;
			}
		}
		else if (dst_y == pos_y) {
			if (dst_x > pos_x) {
				d = 1;
			}
			else {
				d = 0;
			}
		}
		else if (dst_x > pos_x&& dst_y > pos_y) {
			avil_d.push_back(2);
			avil_d.push_back(1);

			d = avil_d[rand() % avil_d.size()];
		}
		else if (dst_x < pos_x && dst_y > pos_y) {
			d = 0;
		}
		else if (dst_x > pos_x&& dst_y < pos_y) {
			d = 3;
		}
		else {
			avil_d.push_back(3);
			avil_d.push_back(0);
			d = avil_d[rand() % avil_d.size()];
		}

		break;

	case 5://///////////////////////////////////mesh odd-even with q learning
		if (e_x == 0 && e_y == 0) {
			return 0;
		}

		if (e_x == 0)
		{
			if (e_y > 0)
			{
				avil_d.push_back(2);														//x=0, y>0  UP
			}
			else {
				avil_d.push_back(3);														//x=0, y<0  DOWN
			}
		}
		else
		{
			if (e_x > 0) {
				if (e_y == 0) {
					avil_d.push_back(1);													//x>0, y=0  RIGHT
				}
				else
				{
					if (dst_x % 2 == 1 || e_x != 1) {
						avil_d.push_back(1);												//x>0, y!=0, x!=1 or dst is odd  RIGHT
					}
					if ((pos_x % 2 == 1) || pos_x == src_x) {
						if (e_y > 0) {
							avil_d.push_back(2);											//x>0, y>0, pos_x is odd or pos_x is src  UP
						}
						else {
							avil_d.push_back(3);											//x>0, y<0, pos_x is odd or pos_x is src  DOWN
						}
					}
				}
			}
			else {
				avil_d.push_back(0);														//x<0  LEFT
				if (pos_x % 2 == 0) {
					if (e_y > 0) {
						avil_d.push_back(2);												//x<0, pos_xis even, y>0  UP
					}
					else
					{
						avil_d.push_back(3);												//x<0, pos_x is even, y<=0  DOWN
					}
				}
			}
		}
		d = avil_d[0];
		temp_value_q = qtable[dst][d];
		for (int i = 1; i < avil_d.size(); i++) {
			direction = avil_d[i];

			if ((qtable[dst][direction]) < temp_value_q) {
				d = direction;
				temp_value_q = qtable[dst][d];
			}
		}
		break;

	case 6://///////////////////////////////////////random minimum path
		if (pos_x != dst_x) {
			if (pos_x < dst_x) {
				avil_d.push_back(RIGHT);
			}
			else if (pos_x > dst_x) {
				avil_d.push_back(LEFT);
			}

		}
		if (pos_y != dst_y) {
			if (pos_y < dst_y) {
				avil_d.push_back(UP);
			}
			else if (pos_y > dst_y) {
				avil_d.push_back(DOWN);
			}

		}

		d = avil_d[rand() % avil_d.size()];
		break;

	default:
		d = TO_PROCESSOR;
	}
	if (d < 0) {
		assert(0);
	}
	return d;
		/*int dst=head_flit.dest;
		int dst_x=-1;
		int dst_y=-1;
		get_xy(dst, dst_x, dst_y);

		int d = -1;
		if (pos_x < dst_x) {
			d = RIGHT;
		}
		else if (pos_x > dst_x) {
			d = LEFT;
		}
		else {
			if (pos_y < dst_y) {
				d = UP;
			}
			else if (pos_y > dst_y) {
				d = DOWN;
			}
			else {
				d = TO_PROCESSOR;
				
			}
		}


		if (d<0) {
			assert(0);
		}
		return d;
		*/
	}

/*
int Router::routing_computation(flit_type head_flit)
	{
		int dst=head_flit.dest;
		int dst_x=-1;
		int dst_y=-1;
		get_xy(dst, dst_x, dst_y);

		int d = -1;
		if (pos_x < dst_x) {
			if ((dst_x - pos_x) < X_change_road_threshold) {
				d = RIGHT;
			}
			else {
				d =LEFT;
			}
		}
		else if (pos_x > dst_x) {
			if ((pos_x - dst_x) < X_change_road_threshold) {
				d = LEFT;
			}
			else {
				d = RIGHT;
			}
		}
		else {
			if (pos_y < dst_y) {
				 if ((dst_y - pos_y) < Y_change_road_threshold) {
				 	d = UP;
				 }
				 else {
				 	d = DOWN;
				 }
			}
			else if (pos_y > dst_y) {
				if ((pos_y - dst_y) < Y_change_road_threshold) {
					d = DOWN;
				}
				else {
					d = UP;
				}
			}
			else {
				d = TO_PROCESSOR;
				
			}
		}


		if (d<0) {
			assert(0);
		}
		return d;
		
	}
*/
void Router::input_port()
{
	
	while(1)
	{
		//cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] input port"<<endl;
		wait(SC_ZERO_TIME);
		for(int input_id=0;input_id<port_no;input_id++)
		{
			if (input_id != TO_PROCESSOR)
			{
				if (output_backward_data[input_id].updated_latency_flag == true)///////////////////////////////////////If updating signal is detected then update Q table
				{
					int temp_updated = 0.5 * output_backward_data[input_id].updated_latency + 0.5 * qtable[output_backward_data[input_id].updated_latency_dst][input_id];
					if (temp_updated >= 65535)
						qtable[output_backward_data[input_id].updated_latency_dst][input_id] = 65535;
					else
						qtable[output_backward_data[input_id].updated_latency_dst][input_id] = temp_updated;
					//qtable[output_backward_data[input_id].updated_latency_dst][input_id] = (int)0.5 * output_backward_data[input_id].updated_latency + 0.5 * qtable[output_backward_data[input_id].updated_latency_dst][input_id];//Update Q table
					output_backward_data[input_id].updated_latency_flag = false;/////////////////////////////////////After updating Q table, reset flag
				}
			}
			

			for(int i=0;i<vc_number;i++)
			{
				old_read_pointer.at(input_id).at(i)=read_pointer.at(input_id).at(i);
			}

			switch(input_port_state[input_id])
			{
			case vc_arbitration: //vc_arbitration
				{
					
					//cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] input port vc_arbitration input_id="<<input_id<<endl;
					
					//first we need make sure we reset ready signal if we just sent data out one cycle ago
					//we also make sure, output port is not sending data !!!--this can be sured						
					if(true == reset_ready_signal.at(input_id))
					{
						reset_ready_signal.at(input_id)=false;
						router_forward_interface temp_output_forward_data;
						temp_output_forward_data.ready=false;
						output_forward_interface[ current_output_direction.at(input_id) ].write(temp_output_forward_data);
					
					}
					//now we find available vc
					
					for(int i=0;i<vc_number;i++)
					{
						//round robin
						int vc_id=(i+vc_robin.at(input_id))%vc_number;
						

						//empty
						if(true == vc_state[input_id][vc_id].empty)
						{
							/*cout<<sc_simulation_time()<<" input_id="<<input_id<<" vc="<<vc_id << "empty. With vc reserved="
								<<vc_state[input_id][vc_id].reserved<<" vc_available="<<vc_state[input_id][vc_id].available_for_new<<endl;*/
							continue;		
							
						}
						//not ready because of routing delay
						else if(true == buffer[input_id][vc_id][read_pointer.at(input_id).at(vc_id)].head &&
							 0 < buffer[input_id][vc_id][read_pointer.at(input_id).at(vc_id)].routing_delay)
						{
							//cout<<sc_simulation_time()<<" input_id="<<input_id<<" vc="<<vc_id << "delayed by routing algorithm"<<endl;
							continue;
							
						}
						//ready
						else
						{
							//important here, we update output port id in this step
							if(true == buffer[input_id][vc_id][read_pointer.at(input_id).at(vc_id)].head )
							{
								vc_state[input_id][vc_id].output_port_direction=buffer[input_id][vc_id][read_pointer.at(input_id).at(vc_id)].direction;
								//receive_time[input_id] = double(sc_time_stamp().to_double() / 1000);////////////////////////////////////////////record the head flit's receive time 
							}

							current_output_direction.at(input_id)=vc_state[input_id][vc_id].output_port_direction;
							
							//if aim output port is busy, do not send data out
							//this is helpful though may not be implemented here for complexity of hardware!!!
							
							if(input_granted == output_port_state[current_output_direction.at(input_id)])
							{
								/*cout<<sc_simulation_time()<<" input_id="<<input_id<<" vc="<<vc_id << "try to request but output="<<current_output_direction.at(input_id)
									<<"is sending data from "<< granted_input.at(current_output_direction.at(input_id))
									<<" vc="<<current_vc_id.at( granted_input.at(current_output_direction.at(input_id)) ) << endl;
								*/
								continue;
							}
							else
							{// match and ready vc is found, break the loop and wait grant
								//cout<<sc_simulation_time()<<" input_id="<<input_id<<" vc="<<vc_id << "try to request to output="<<current_output_direction.at(input_id);
								current_vc_id.at(input_id)=vc_id;
								vc_robin.at(input_id)=(vc_id+1)%vc_number;
								input_port_state[input_id]=wait_grant;
								//cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] input port vc_arbitration5input_id="<<input_id<<" vc_id= "<<vc_id<<endl;
								break;
							}
						}
					}

					break;
				}
			case wait_grant:
				{		
					//granted   
					if( request_success[ current_output_direction.at(input_id) ][input_id ] ==true )
					{
						if (input_id == granted_input.at(current_output_direction.at(input_id))&& input_granted == output_port_state[current_output_direction.at(input_id)])
						{
						}
						else
						{
							cout<<sc_simulation_time()<<"error in grant!!"<<endl;
						}

						input_port_state[input_id]=sending_data;
						//since it is granted, we can send data out immediately
						//NOtice:: we assume output port has checked the receiver end for us already!!!

						int current_vc=current_vc_id.at(input_id);
						flit_type to_send_flit=buffer[input_id][current_vc][read_pointer.at(input_id).at(current_vc)];
						int current_next_vc=vc_state[input_id][current_vc].next_vc_id;

						if(true==vc_state[input_id][current_vc].empty)
						{
							cout<<"error!! input request!!"<<endl;
						}
						
						//if it is a head flit, find available vc 
						if (true == to_send_flit.head)
						{
							if (current_output_direction.at(input_id) == TO_PROCESSOR)//////////////////at last step, when arrive at processor, use any available vc rather than vc choice
							{
								int temp_next_vc = output_backward_data[current_output_direction.at(input_id)].available_vc;
								if (-1 == temp_next_vc || true == output_backward_data[current_output_direction.at(input_id)].buffer_full[temp_next_vc])
								{
									cout << "error!! output is not good!!" << endl;
									input_port_state[input_id] = vc_arbitration;
									break;//no vc, go back to arbitration rightaway
								}
								//available vc is got and stored
								else
								{
									int temp_reserve = output_vc_reserve.at(current_output_direction.at(input_id)).at(temp_next_vc);
									if (-1 != temp_reserve)
									{
										cout << sc_simulation_time() << "error!! output is really not good!!" << endl;
										exit(0);
									}
									output_vc_reserve.at(current_output_direction.at(input_id)).at(temp_next_vc) = input_id;

									vc_state[input_id][current_vc].next_vc_id = temp_next_vc;
									current_next_vc = temp_next_vc;
									stable_output_next_vc[input_id][current_vc] = temp_next_vc;
									if (input_id != TO_PROCESSOR)
									{
										leave_time[input_id] = double(sc_time_stamp().to_double() / 1000);///////////////////record the leave time when head flit finds its output port vc
										updated_latency_flag[input_id] = true;/////////////////////Head flit has found corresponding channel and it is available, update flag of Q pacekt
										updated_latency_dst[input_id] = to_send_flit.dest;///////////////////////Found dest
										if (to_send_flit.direction == TO_PROCESSOR)
										{
											check_if_arrived_dst[input_id] = true;///////////////////////////To judge how to update Q packet
										}
										else
										{
											check_if_arrived_dst[input_id] = false;
										}
									}

								}
							}
							else {//////////////////////////////////next node is not processor, follow the vc choice to avoid deadlock
								//if (output_backward_data[current_output_direction.at(input_id)].available_vc == to_send_flit.vc_choice)//////if available_vc is exactly the deadlock-free vc, we choose it
								//{
								//	int temp_next_vc = output_backward_data[current_output_direction.at(input_id)].available_vc;
								int temp_num_available_vc = output_backward_data[current_output_direction.at(input_id)].num_available_vc;
								int temp_next_vc = -1;
								if (temp_num_available_vc == 1)
									temp_next_vc = output_backward_data[current_output_direction.at(input_id)].available_vc;
								else if (temp_num_available_vc == 2)
									temp_next_vc = to_send_flit.vc_choice;

								if (temp_next_vc != to_send_flit.vc_choice)////////////////////////////////////if next vc is not vc_choice, error!!
								{
									cout << "error!!vc choice is not good!!" << endl;
									input_port_state[input_id] = vc_arbitration;
									break;
								}
								//int temp_next_vc = to_send_flit.vc_choice;////////////////////////////////

								//no available vc or the available vc's buffer is full
								else
									if (-1 == temp_next_vc || true == output_backward_data[current_output_direction.at(input_id)].buffer_full[temp_next_vc])
									{
										cout << "error!! output is not good!!" << endl;
										input_port_state[input_id] = vc_arbitration;
										break;//no vc, go back to arbitration rightaway
									}
								//available vc is got and stored
									else
									{
										int temp_reserve = output_vc_reserve.at(current_output_direction.at(input_id)).at(temp_next_vc);
										if (-1 != temp_reserve)
										{
											cout << sc_simulation_time() << "error!! output is really not good!!" << endl;
											exit(0);
										}
										output_vc_reserve.at(current_output_direction.at(input_id)).at(temp_next_vc) = input_id;

										vc_state[input_id][current_vc].next_vc_id = temp_next_vc;
										current_next_vc = temp_next_vc;
										stable_output_next_vc[input_id][current_vc] = temp_next_vc;
										if (input_id != TO_PROCESSOR)
										{
											leave_time[input_id] = double(sc_time_stamp().to_double() / 1000);/////////////////////record the leave time when head flit finds its output port vc
											updated_latency_flag[input_id] = true;/////////////////////////////Head flit has found corresponding channel and it is available, update flag of Q pacekt
											updated_latency_dst[input_id] = to_send_flit.dest;////////////////////////////////Found dest
											if (to_send_flit.direction == TO_PROCESSOR)
											{
												check_if_arrived_dst[input_id] = true;/////////////////////////To judge how to update Q packet
											}
											else
											{
												check_if_arrived_dst[input_id] = false;
											}
										}

									}
							}
							/*}*/
							/*else
							{
								input_port_state[input_id] = vc_arbitration;
								break;
							}*/

						}
						else //not head, we need find available buffer
						{
							
							if(true == output_backward_data[ current_output_direction.at(input_id) ].buffer_full[current_next_vc])
							{
								cout<<"error!! output is not good2!!"<<endl;
								input_port_state[input_id]=vc_arbitration;
								break;//no buffer, go back to arbitration rightaway
							}
						}
					
						//now, we send data out
						router_forward_interface temp_output_forward_data;
						temp_output_forward_data.data_flit=to_send_flit;
						temp_output_forward_data.ready=true;
						temp_output_forward_data.vc_id=current_next_vc;
						output_forward_interface[ current_output_direction.at(input_id) ].write(temp_output_forward_data);

						/*router_log<<sc_simulation_time()<<"granted and send data by input_id= "<<input_id <<" vc= "<<current_vc_id.at(input_id)
						<<"to next_vc="<<current_next_vc
						<<" through output id="<<current_output_direction.at(input_id) 
						<<" seq="<<to_send_flit.packet_sequence
						<<" core "<<to_send_flit.src<<"->"<<to_send_flit.dest<<"successfully"<<endl;*/
						

						if( current_output_direction.at(input_id) != stable_output_direction[input_id][current_vc] || 
							current_next_vc != stable_output_next_vc[input_id][current_vc] )
						{
							cout<<" error in sending data"<<endl;
						}

						read_pointer.at(input_id).at(current_vc)=(read_pointer.at(input_id).at(current_vc)+1)%vc_depth;
						
						//Lastly, we check the flit is tail or not 
						//Notice: it could be both head & tail
						if(true == to_send_flit.tail)
						{
							/*cout<<sc_simulation_time()<<"TAIl!!--granted and send data by input_id= "<<input_id <<" vc= "<<current_vc_id.at(input_id)
						<<"to next_vc="<<current_next_vc
						<<" through output id="<<current_output_direction.at(input_id) 
						<<" seq="<<to_send_flit.packet_sequence
						<<" core "<<to_send_flit.src<<"->"<<to_send_flit.dest<<endl;
							*/
							vc_state[input_id][current_vc].reserved=false;
							vc_state[input_id][current_vc].available_for_new=true;
							input_port_state[input_id]=vc_arbitration;
							//Important here: we need reset data ready signal at next clk cycle
							reset_ready_signal.at(input_id)=true;

							//update next vc state
							output_vc_reserve.at( current_output_direction.at(input_id) ).at(current_next_vc)= -1;

							break;

						}


					}
					// failed //(input_id != granted_input.at(current_output_direction.at(input_id)) && input_granted == output_port_state[current_output_direction.at(input_id)])
					else if( request_fail[ current_output_direction.at(input_id) ][input_id ] ==true )
					{
						input_port_state[input_id]=vc_arbitration;
						//cout<<sc_simulation_time()<<" input_id="<<input_id<<" rejected by output="<< current_output_direction.at(input_id) <<endl;
					}
					else
					{
					//just wait
						//cout<<sc_simulation_time()<<" input_id="<<input_id<<" wait grant to output="<< current_output_direction.at(input_id) <<endl;
					}
					break;
				}
			case sending_data:
				{
					//cout<<sc_simulation_time()<<" input_id="<<input_id<<" send data to output="<< current_output_direction.at(input_id) <<endl;

					//we should make sure there is data to send!
					int current_vc=current_vc_id.at(input_id);
					if(vc_state[input_id][current_vc].empty==true)
					{
						input_port_state[input_id]=vc_arbitration;
						//Important here: we need reset data ready signal immediately because ready signal is still on by previouse flit
							router_forward_interface temp_output_forward_data;
							temp_output_forward_data.ready=false;
							output_forward_interface[ current_output_direction.at(input_id) ].write(temp_output_forward_data);
							break;//no vc, go back to arbitration rightaway

					}

					flit_type to_send_flit=buffer[input_id][current_vc][read_pointer.at(input_id).at(current_vc)];
					int current_next_vc=vc_state[input_id][current_vc].next_vc_id;
					
					//if it is a head flit, find available vc 
					if(true == to_send_flit.head)
					{
						if (current_output_direction.at(input_id) == TO_PROCESSOR)
						{
							int temp_next_vc = output_backward_data[current_output_direction.at(input_id)].available_vc;
							if (-1 == temp_next_vc || true == output_backward_data[current_output_direction.at(input_id)].buffer_full[temp_next_vc])
							{
								input_port_state[input_id] = vc_arbitration;

								/*cout<<sc_simulation_time()<<"send data by input_id= "<<input_id <<" vc= "<<current_vc_id.at(input_id)
							<<"to next_vc="<<current_next_vc
							<<" through output id="<<current_output_direction.at(input_id)
							<<" seq="<<to_send_flit.packet_sequence
							<<" core "<<to_send_flit.src<<"->"<<to_send_flit.dest<<"BUT FAILED because of no VC?? OR buffer _full?????????????????????????????????? "<<endl;
								*/
								//Important here: we need reset data ready signal immediately because ready signal is still on by previouse flit
								router_forward_interface temp_output_forward_data;
								temp_output_forward_data.ready = false;
								output_forward_interface[current_output_direction.at(input_id)].write(temp_output_forward_data);
								break;//no vc, go back to arbitration rightaway
							}
							//available vc is got and stored
							else
							{
								int temp_reserve = output_vc_reserve.at(current_output_direction.at(input_id)).at(temp_next_vc);
								if (-1 != temp_reserve)
								{
									cout << sc_simulation_time() << "error!! output is really not good2!!" << endl;
									exit(0);
								}

								output_vc_reserve.at(current_output_direction.at(input_id)).at(temp_next_vc) = input_id;

								vc_state[input_id][current_vc].next_vc_id = temp_next_vc;
								current_next_vc = temp_next_vc;
							}
						}
						else {
							int temp_num_available_vc = output_backward_data[current_output_direction.at(input_id)].num_available_vc;
							int temp_next_vc = -1;
							if (temp_num_available_vc == 1)
								temp_next_vc = output_backward_data[current_output_direction.at(input_id)].available_vc;
							else if (temp_num_available_vc == 2)
								temp_next_vc = to_send_flit.vc_choice;
							//int temp_next_vc = output_backward_data[  current_output_direction.at(input_id)  ].available_vc;
							if (temp_next_vc != to_send_flit.vc_choice)////////////////////////////if next vc is not vc_choice, error!!
							{
								cout << "error!!vc choice is not good!! in sending data" << endl;
								input_port_state[input_id] = vc_arbitration;
							}
							//int temp_next_vc = to_send_flit.vc_choice;////////////////////////////

							//no available vc or the available vc's buffer is full
							else
								if (-1 == temp_next_vc || true == output_backward_data[current_output_direction.at(input_id)].buffer_full[temp_next_vc])
								{
									input_port_state[input_id] = vc_arbitration;

									/*cout<<sc_simulation_time()<<"send data by input_id= "<<input_id <<" vc= "<<current_vc_id.at(input_id)
								<<"to next_vc="<<current_next_vc
								<<" through output id="<<current_output_direction.at(input_id)
								<<" seq="<<to_send_flit.packet_sequence
								<<" core "<<to_send_flit.src<<"->"<<to_send_flit.dest<<"BUT FAILED because of no VC?? OR buffer _full?????????????????????????????????? "<<endl;
									*/
									//Important here: we need reset data ready signal immediately because ready signal is still on by previouse flit
									router_forward_interface temp_output_forward_data;
									temp_output_forward_data.ready = false;
									output_forward_interface[current_output_direction.at(input_id)].write(temp_output_forward_data);
									break;//no vc, go back to arbitration rightaway
								}
							//available vc is got and stored
								else
								{
									int temp_reserve = output_vc_reserve.at(current_output_direction.at(input_id)).at(temp_next_vc);
									if (-1 != temp_reserve)
									{
										cout << sc_simulation_time() << "error!! output is really not good2!!" << endl;
										exit(0);
									}

									output_vc_reserve.at(current_output_direction.at(input_id)).at(temp_next_vc) = input_id;

									vc_state[input_id][current_vc].next_vc_id = temp_next_vc;
									current_next_vc = temp_next_vc;
								}
						}
					}
					else //not head, we need find available buffer
					{
						
						if(true == output_backward_data[ current_output_direction.at(input_id) ].buffer_full[current_next_vc])
						{
							/*router_log<<sc_simulation_time()<<"send data by input_id= "<<input_id <<" vc= "<<current_vc_id.at(input_id)
						<<"to next_vc="<<current_next_vc
						<<" through output id="<<current_output_direction.at(input_id) 
						<<" seq="<<to_send_flit.packet_sequence
						<<" core "<<to_send_flit.src<<"->"<<to_send_flit.dest<<"BUT FAILED because of bo buffer"<<endl;
						*/
						
							input_port_state[input_id]=vc_arbitration;
							//Important here: we need reset data ready signal immediately because ready signal is still on by previouse flit
							router_forward_interface temp_output_forward_data;
							temp_output_forward_data.ready=false;
							output_forward_interface[ current_output_direction.at(input_id) ].write(temp_output_forward_data);
							break;//no buffer, go back to arbitration rightaway
						}
					}

					
				
					//now, we send data out
					router_forward_interface temp_output_forward_data;
					temp_output_forward_data.data_flit=to_send_flit;
					temp_output_forward_data.ready=true;
					temp_output_forward_data.vc_id=current_next_vc;
					output_forward_interface[ current_output_direction.at(input_id) ].write(temp_output_forward_data);

					
					/*router_log<<sc_simulation_time()<<"send data by input_id= "<<input_id <<" vc= "<<current_vc_id.at(input_id)
						<<"to next_vc="<<current_next_vc
						<<" through output id="<<current_output_direction.at(input_id) 
						<<" seq="<<to_send_flit.packet_sequence
						<<" core "<<to_send_flit.src<<"->"<<to_send_flit.dest<<"Successfully"<<endl;
					*/
					
						if( current_output_direction.at(input_id) != stable_output_direction[input_id][current_vc] || 
							current_next_vc != stable_output_next_vc[input_id][current_vc] )
						{
							cout<<" error in sending data"<<endl;
						}

					read_pointer.at(input_id).at(current_vc)=(read_pointer.at(input_id).at(current_vc)+1)%vc_depth;
					
					//Lastly, we check the flit is tail or not 
					//Notice: it could be both head & tail
					if(true == to_send_flit.tail)
					{
						/*cout<<sc_simulation_time()<<"send data by input_id= "<<input_id <<" vc= "<<current_vc_id.at(input_id)
						<<"to next_vc="<<current_next_vc
						<<" through output id="<<current_output_direction.at(input_id) 
						<<" seq="<<to_send_flit.packet_sequence
						<<" core "<<to_send_flit.src<<"->"<<to_send_flit.dest<<"TAIL!"<<endl;
						*/
						vc_state[input_id][current_vc].reserved=false;
						vc_state[input_id][current_vc].available_for_new=true;
						input_port_state[input_id]=vc_arbitration;
						//Important here: we need reset data ready signal at next clk cycle
						reset_ready_signal.at(input_id)=true;

						//update next vc state
						output_vc_reserve.at( current_output_direction.at(input_id) ).at(current_next_vc)= -1;
						break;

					}
				
				break;
				}
			}//end switch
		}//end for
		wait();
	}//end while

}// end function of input

void Router::backward_of_input_port()
{
	
	while(1)
	{
		//cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] backward input port"<<endl;
		wait(0.1, SC_NS);
		for(int input_id=0;input_id<port_no;input_id++)
		{
			
			router_backward_interface temp_input_backward_data;
			//checking each VC buffer
			if (input_id != TO_PROCESSOR)
			{
				if (updated_latency_flag[input_id] == true)
				{
					temp_input_backward_data.updated_latency_flag = true;//////The input_id here is the input port that owns head flit and is certain to be able to  send flit. Set its backward flag 1 to let last router know this port has new Q packet
					temp_input_backward_data.updated_latency_dst = updated_latency_dst[input_id];//////////////////////////Update dst
					if (check_if_arrived_dst[input_id] == true)/////////////////////////////Update Q value according to whether it has arrived dst.
					{
						temp_input_backward_data.updated_latency = get_waiting_time(receive_time[input_id], leave_time[input_id]);/////////////////////////Q value writing to packet = waiting_time
					}
					else
					{
						int temp_latency = qtable[input_forward_data[input_id].data_flit.dest][current_output_direction.at(input_id)]+ get_waiting_time(receive_time[input_id], leave_time[input_id]);
						if (temp_latency >= 65535)
							temp_input_backward_data.updated_latency = 65535;
						else
						temp_input_backward_data.updated_latency = temp_latency;///////////////////////Q value writing to packet = waiting_time + Q value in next step
					}
				}
				/*if (temp_input_backward_data.updated_latency != 0) 
				{
					std::cout << qtable[input_forward_data[input_id].data_flit.dest][input_forward_data[input_id].data_flit.direction];
					assert(0);
				}*/
			}
			
			
			for(int vc_id=0;vc_id<vc_number;vc_id++)
			{
				//update available space
				if(old_read_pointer.at(input_id).at(vc_id)!=read_pointer.at(input_id).at(vc_id))
				{
					available_space.at(input_id).at(vc_id)++;
				}
				if(old_write_pointer.at(input_id).at(vc_id)!=write_pointer.at(input_id).at(vc_id))
				{
					available_space.at(input_id).at(vc_id)--;
				}
				//check it is full or near full or not
				//attention here!!! when you notice you are near full, there are data already coming into it. 
					//The total number of data should be: when the ack_buffer_full signal reached sender, how many flits would be sent out?
					//it is : Already sent data(=number of delays on link)+ to_be_sent_data(=the time for the ack to reach the sender=bus_driver_time+proporgation_delay)
			
				int temp_space;
				temp_space = 1; // all links in mesh are 1-cycle delay

				if(temp_space>= available_space.at(input_id).at(vc_id))
				{
					temp_input_backward_data.buffer_full[vc_id]=true;
				//	router_log<<sc_simulation_time()<<"input_id="<<input_id<<" vc_id= "<<vc_id<<" full,sending back!!";
				}
				else
				{
					temp_input_backward_data.buffer_full[vc_id]=false;
				}
				//check it is empty or not
				if(vc_depth == available_space.at(input_id).at(vc_id))
				{
					vc_state[input_id][vc_id].empty=true;
				}
				else
				{
					vc_state[input_id][vc_id].empty=false;
				}

			}

			////checking available VC id
			//This is important!!! This error made me unsleep for one day!!	
			temp_input_backward_data.available_vc = -1;
			temp_input_backward_data.available_vc1 = -1;
			int temp_num_available_vc = 0;
			//check the number of available vc///////////////////////////////
			if ((true == vc_state[input_id][0].available_for_new) && (true == vc_state[input_id][1].available_for_new))
				temp_num_available_vc = 2;
			else if ((true == vc_state[input_id][0].available_for_new) || (true == vc_state[input_id][1].available_for_new))
				temp_num_available_vc = 1;
			else temp_num_available_vc = 0;

			temp_input_backward_data.num_available_vc = temp_num_available_vc;

			if(temp_num_available_vc ==1)
				for (int vc_id = 0; vc_id < vc_number; vc_id++)
				{
					//set backward VC id
					if (true == vc_state[input_id][vc_id].available_for_new)
					{
						temp_input_backward_data.available_vc = vc_id;
						break;
					}
				}
			else if (temp_num_available_vc == 2)
			{
				temp_input_backward_data.available_vc = 0;
				temp_input_backward_data.available_vc1 = 1;
			}
			

			//after checking Vc, send vc state back
			input_backward_interface[input_id].write(temp_input_backward_data);

		}

		wait();
	}

}
void Router::output_port()
{
	
	while(1)
	{
		//cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] output_port"<<endl;
		wait(SC_ZERO_TIME);
		for(int output_id=0;output_id<port_no;output_id++)
		{
			//Clear all grant signal at first, the signal would be received by input port because input ports are listenning these signals and the signal 
			//would not be updated untill delta cycle
			for(int input_id=0;input_id<port_no;input_id++)
			{
					request_success[output_id][input_id]=false;
					request_fail[output_id][input_id]=false;

			}

			switch(output_port_state[output_id])
			{
			case input_arbitration:
				{
					//cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] output port arbitrating "<<"output id="<<output_id<<endl;
					//check each input port
					for(int i=0;i<port_no;i++)
					{//round robin
						int input_id=(i+input_robin.at(output_id))%port_no;
						//assume all input ports are not granted first
						granted_input.at(output_id)=-1;
						//No request from input port: skip
						if(input_port_state[input_id]!=wait_grant || current_output_direction.at(input_id) != output_id)
						{
							continue;
						}
						//get to send flit from input port
						
						

						flit_type temp_data=buffer[input_id][current_vc_id.at(input_id)][ read_pointer.at(input_id).at( current_vc_id.at(input_id)) ];
						//if it is head, find available vc and make sure this vc is not full
						if(true == temp_data.head)
						{
							if (output_id==TO_PROCESSOR)
							{
								int available_vc = output_backward_data[output_id].available_vc;
								if (find(output_vc_reserve.at(output_id).begin(), output_vc_reserve.at(output_id).end(), -1) == output_vc_reserve.at(output_id).end())
								{
									request_fail[output_id][input_id] = true;
									continue;

								}
								else if (0 > available_vc) //maybe the receiver do not want new packet to come///////////////////////////
								{
									request_fail[output_id][input_id] = true;
									continue;
								}
								//this branch may taken when VC0 is occupied but VC 1 is not
								//However, the receiver may say VC0 is available because of the delay on link
								else if (-1 != output_vc_reserve.at(output_id).at(available_vc))//////////////////////////////
								{
									request_fail[output_id][input_id] = true;
									continue;
								}
								else
								{// check whether this vc is full
									if (false == output_backward_data[output_id].buffer_full[available_vc])/////////////////////////
									{
										request_success[output_id][input_id] = true;
										//cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"Successed for head"<<endl;
										output_port_state[output_id] = input_granted;
										granted_input.at(output_id) = input_id;
										input_robin.at(output_id) = (input_id + 1) % port_no;
										break;
										//}
									}
									else
									{
										request_fail[output_id][input_id] = true;
										//	cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"failed for no head buffer"<<endl;
										continue;
									}
								}
							}
							else {
								int available_vc = output_backward_data[output_id].available_vc;
								int temp_num_available_vc = output_backward_data[output_id].num_available_vc;///////////////////////////////
								if (temp_num_available_vc == 0)
								{
									request_fail[output_id][input_id] = true;
									continue;
								}
								else if (temp_num_available_vc == 1)
								{
									if (available_vc != temp_data.vc_choice)
									{
										request_fail[output_id][input_id] = true;
										continue;
									}
									else if (find(output_vc_reserve.at(output_id).begin(), output_vc_reserve.at(output_id).end(), -1) == output_vc_reserve.at(output_id).end())
									{
										request_fail[output_id][input_id] = true;
										continue;

									}
									else if (0 > available_vc) //maybe the receiver do not want new packet to come///////////////////////////
									{
										request_fail[output_id][input_id] = true;
										continue;
									}
									//this branch may taken when VC0 is occupied but VC 1 is not
									//However, the receiver may say VC0 is available because of the delay on link
									else if (-1 != output_vc_reserve.at(output_id).at(available_vc))///////////////////////////////
									{
										request_fail[output_id][input_id] = true;
										continue;
									}
									else
									{// check whether this vc is full
										if (false == output_backward_data[output_id].buffer_full[available_vc])////////////////////////
										{
											request_success[output_id][input_id] = true;
											//cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"Successed for head"<<endl;
											output_port_state[output_id] = input_granted;
											granted_input.at(output_id) = input_id;
											input_robin.at(output_id) = (input_id + 1) % port_no;
											break;
											//}
										}
										else
										{
											request_fail[output_id][input_id] = true;
											//	cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"failed for no head buffer"<<endl;
											continue;
										}
									}
								}
								//if (available_vc != temp_data.vc_choice)/////////////////////////first we check if it is vc_choice
								//{
								//	request_fail[output_id][input_id] = true;///////////////////////not vc_choice, fail
								//	continue;
								//}
									//no available vc
								else if (temp_num_available_vc == 2)
								{
									int temp_vc_choice = temp_data.vc_choice;
									if (find(output_vc_reserve.at(output_id).begin(), output_vc_reserve.at(output_id).end(), -1) == output_vc_reserve.at(output_id).end())
									{
										request_fail[output_id][input_id] = true;
										continue;

									}
									//else if ((0 > available_vc) && (0 > available_vc1)) //maybe the receiver do not want new packet to come//////////////////
									//{
									//	//cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"failed for no VC"<<endl;

									//	//
									//	request_fail[output_id][input_id] = true;

									//	continue;
									//}
									//this branch may taken when VC0 is occupied but VC 1 is not
									//However, the receiver may say VC0 is available because of the delay on link
									else if (-1 != output_vc_reserve.at(output_id).at(temp_vc_choice)) ////////////////////
									{
										request_fail[output_id][input_id] = true;

										continue;

									}
									else
									{// check whether this vc is full
										if (false == output_backward_data[output_id].buffer_full[temp_vc_choice]) ////////////////////
										{
											//grant input port
											/*if(available_vc!= temp_data.vc_choice)
											{
												request_fail[output_id][input_id] = true;
												continue;
											}
											else
											{*/
											request_success[output_id][input_id] = true;
											//cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"Successed for head"<<endl;
											output_port_state[output_id] = input_granted;
											granted_input.at(output_id) = input_id;
											input_robin.at(output_id) = (input_id + 1) % port_no;
											break;
											//}
										}
										else
										{
											request_fail[output_id][input_id] = true;
											//	cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"failed for no head buffer"<<endl;
											continue;
										}
									}
								}
							}
						}
						// if it is not a head, just check buffer[next_vc]
						else
						{
							//get next vc id from mapping table
							int next_vc=vc_state[input_id][current_vc_id.at(input_id)].next_vc_id;
							if(false == output_backward_data[output_id].buffer_full[next_vc])
							{//if not full, grant it
								request_success[output_id][input_id]=true;

							//	cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"Successed for body"<<endl;
								
								output_port_state[output_id]=input_granted;
								granted_input.at(output_id)=input_id;
								input_robin.at(output_id)=(input_id+1)%port_no;
								break;
							}
							else
							{
								request_fail[output_id][input_id]=true;
							//	cout<<sc_simulation_time()<<"output id="<<output_id<<"output port arbitrating request from input_port="<<input_id<<"failed for no Buffer"<<endl;
									
								continue;
							}
						}

					}

					break;
				}
			case input_granted:
				{
					//if current input shift to vc_arbitration, meaning that it does not want to send data more
					//we release this output port
					//cout<<sc_simulation_time()<<" router ["<<node_level<<","<<node_rank<<"] outputid= "<<output_id
					//	<<" grant input="<<granted_input.at(output_id)<<endl;
					

					if(vc_arbitration == input_port_state[  granted_input.at(output_id) ])
					{
						output_port_state[output_id]=input_arbitration;
						granted_input.at(output_id)=-1;

					}

					break;
				}

			}
		}
		wait();
	}
}
int Router::get_waiting_time(double& receive_time, double& leave_time)////////////////////calculate the local waiting time
{
	double waiting_time = leave_time - receive_time;
	if (waiting_time <= 3 * AMS)
		return 0;
	else if (waiting_time <= 9 * AMS)
		return 1;
	else if (waiting_time <= 27 * AMS)
		return 2;
	else
		return 3;
	receive_time = 0;
	leave_time = 0;
}

//int Router::vc_computation(flit_type head_flit, int input_port, int output_port, int old_vc)//////////////////////
//{
	//version 1: ChaoSheng: left right has 1 vc, up down has 2 vc
	/*int vc_choice = -1;
	int old_vc_choice = old_vc;
	int dst = head_flit.dest;

	int dst_x = -1;
	int dst_y = -1;

	get_xy(dst, dst_x, dst_y);

	if (input_port == LEFT)
	{
		vc_choice = 0;
	}
	else if (input_port == RIGHT)
	{
		if ((output_port == LEFT) || (output_port == TO_PROCESSOR))
			vc_choice = 0;
		else vc_choice = 1;
	}
	else if (input_port == UP)
	{
		if ((output_port == LEFT) || (output_port == RIGHT))
			vc_choice = 0;
		else vc_choice = old_vc_choice;
	}
	else if (input_port == DOWN)
	{
		if ((output_port == LEFT) || (output_port == RIGHT))
			vc_choice = 0;
		else vc_choice = old_vc_choice;
	}
	else
	{
		if ((output_port == LEFT) || (output_port == RIGHT))
			vc_choice = 0;
		else
		{
			if (dst_x == pos_x)
				vc_choice = rand() % 2;
			else if (dst_x > pos_x)
				vc_choice = 0;
			else vc_choice = 1;
		}
	}
	return vc_choice;*/


	//version 2: W.Dally page4: based on the no of nodes, comparing dst and src
	/*int vc_choice = 0;
	int dst = head_flit.dest;

	int dst_x = -1;
	int dst_y = -1;

	get_xy(dst, dst_x, dst_y);

	if (pos_y < dst_y)
		vc_choice = 1;
	else if ((pos_y == dst_y) && (pos_x < dst_x))
		vc_choice = 1;
	else if ((pos_x == dst_x) && (pos_y == dst_y))
		vc_choice = rand() % 2;
	else vc_choice = 0;

	return vc_choice;*/


	//version 3: W.Dally page3: only change vc in one corner
	/*int vc_choice = -1;
	int old_vc_choice = old_vc;

	if ((input_port == LEFT)||(input_port == RIGHT))
	{
		if (output_port == UP)
		{
			if (old_vc_choice == 0)
				vc_choice = 1;
			else vc_choice = 0;
		}
		else vc_choice = old_vc_choice;
	}
	else vc_choice = old_vc_choice;
	
	return vc_choice;*/


	//version 4: W.Dally decide vc in every node, comparing pos and next node.
	/*int vc_choice = 0;

	if (output_port == 1 || output_port == 2)
		vc_choice = 1;
	else if (output_port == 4)
		vc_choice = rand() % 2;
	else vc_choice = 0;

	return vc_choice;*/
//}

#endif