#ifndef electronic_processor2
#define electronic_processor2

#include "electronic_mesh_definition.h"
#include "stdio.h"
#include "math.h"
#include "time.h"
#include "stdlib.h"
#include "global_data.h"
//these numbers are used for rnd
#define A 16807.0
#define M 2147483647.0
#define indifinite 4294967296
static double r_seed;

void generate_seed()
{
    srand((unsigned)time(NULL)); 
    r_seed=rand()%10; 
	if(r_seed==0)
		r_seed=1;
   
}



double rnd()//generate uniform distribution random in(0,1)
// random is stored in r_seed
{
	
	r_seed=fmod(A*r_seed,M);
//	printf("%lf\n",r_seed*4.656612875e-10);
	double k=r_seed*4.656612875e-10;
	return(k);
}

double gen_exp_dis_time(double a)//generate exponantial variable with parameter a
{
    return (-log(rnd())/a);
}




SC_MODULE(electronic_processor)
{
	sc_in<bool>   clk ;
	sc_out<router_forward_interface> output_forward_interface;
	sc_in<router_backward_interface> output_backward_interface;

	sc_in<router_forward_interface> input_forward_interface;
	sc_out<router_backward_interface> input_backward_interface;

	int id;
	int pos_x;
	int pos_y;
	int req_count;
	int tear_down_count;
	int buffer_req_in;
	int buffer_tear_down_in;
	double total_packet;
	double	total_delay;
	double total_power;

	//flit_type gen_packet[infinite];
	flit_type *gen_packet;
	
	int req_read;
	int req_write;
	
	int next_time_remain;
	int tear_time_remain;
	int processor_state;
	
	int receive_previous_packet_remain;
	int send_previous_packet_remain;
	bool packet_being_sent;

	int packet_sequence;
	int clear_write_signal_time_remain;

	int wait_packet_transmission;
	int current_vc;

	flit_type temp_packet;

	enum sending_state_type {free, core_sending_data};
	sending_state_type sending_state;

	//ofstream send_req_out_log;

	void update_remain_time();	
	void receive_packet();
	void send_request();
	void initial();
    void setid(int num);
    int getid();
    int getaddress(int x, int y);
    void setpos(int x, int y);
    void getpos(int &x, int &y);
    void get_xy(int id, int &x, int &y);

	SC_HAS_PROCESS(electronic_processor);
	electronic_processor(sc_module_name name=sc_gen_unique_name("core")) : sc_module(name)
	{
		
		//send_req_out_log.open("send_req_out_log_test.txt");
		//initial
		generate_seed();
		//r_seed=1;
		gen_exp_dis_time(injection_rate);
		
		total_packet=0;
		total_delay=0;
		total_power=0;

		gen_packet= new flit_type [infinite];
		

		req_count=0;
		packet_sequence=0;
		req_write=0;
		req_read=0;
		buffer_req_in=0;
		buffer_tear_down_in=0;
		processor_state=0;
		receive_previous_packet_remain=0;
		send_previous_packet_remain=0;

		id=-1;
		pos_x=-1;
		pos_y=-1;
		packet_being_sent=false;

		//double temp=gen_exp_dis_time(injection_rate);
		//next_time_remain=(int) temp*pkt_size;
		next_time_remain=0;	
		clear_write_signal_time_remain=0;
		sending_state=free;
		wait_packet_transmission=0;
			


		

		//initial();

		SC_METHOD(update_remain_time);
		sensitive<<clk.pos();

		SC_THREAD(send_request);
		sensitive<<clk.pos();	

		SC_METHOD(receive_packet);
		sensitive<<clk.pos();


	}
};


void electronic_processor::setid(int num) {
	id = num;
}
int electronic_processor::getid() {
	return id;
}
int electronic_processor::getaddress(int x, int y) {
	return (y*NOC_WIDTH + x);
}
void electronic_processor::setpos(int x, int y) {
	pos_x = x;
	pos_y = y;
}
void electronic_processor::getpos(int &x, int &y) {
	x = pos_x; 
	y = pos_y;
}
void electronic_processor::get_xy(int id, int &x, int &y) {
	y = id / NOC_WIDTH;
	x = id % NOC_WIDTH;
	assert (id == y*NOC_WIDTH + x);
}
void electronic_processor::update_remain_time()
{
		//cout<<"in update_remain_time"<<endl;
		double temp;
		int dest_temp;
		int vc_temp;
//		int src_chip;
	


		if(next_time_remain>0)
			next_time_remain--;
		if(next_time_remain==0)
		{
				
			temp=gen_exp_dis_time(injection_rate);
				//float convert_rate=32/12.5;
			next_time_remain=int (temp*const_pkt_length);

			if (rand() % 10 == 0) /////////////////////////////////////////////////////////////////////////////////hotspot
			{
				dest_temp =7;
			}
			else 
			{
				dest_temp = rand() % processor_no;
			}
			while (dest_temp == id)
			{
				dest_temp = int(rnd() * processor_no);
			}

			//dest_temp=int (rnd()*processor_no);/////////////////////////////////////////////////////////////uniform
			//while(dest_temp==id)
			//{
			//	dest_temp=int( rnd()*processor_no);
			//}

			if (dest_temp > id)////////////////////////////////////////////////////////////////////////////////////if dest>id, we choose 1 vc; if dest<id, we choose 0 vc
			{
				vc_temp = 1;
			}
			else if (dest_temp < id)
			{
				vc_temp = 0;
			}
				
				//dest_temp=11;
				if((req_write+1)%infinite==req_read)
				{
					//if it is full, we just ignore new generated pkt, no throughput will be impacted
					//while, delay is impacted a little.
					//however, it is not important, since it is already saturated
					//cout<<"buffer full in processor"<<endl;
				}
				else
				{
					gen_packet[req_write].power=0;
					gen_packet[req_write].dest = dest_temp;
					gen_packet[req_write].src=id;
					gen_packet[req_write].vc_choice = vc_temp;

					gen_packet[req_write].packet_length=const_pkt_length;
					gen_packet[req_write].packet_sequence=packet_sequence;
					gen_packet[req_write].generate_time=int (sc_time_stamp().to_double()/1000);//to ns
					gen_packet[req_write].waiting_time=0;
				
					int qr;
					qr=int(sc_simulation_time());
					send_req_out_log<<"-1 "<<packet_sequence<<" "<<id<<" "<<dest_temp<<" "<<qr<<endl;

					packet_sequence++;
					req_write=(req_write+1)%infinite;
				
				}
				
			}
		

	//	wait();
}
void electronic_processor::send_request()
{
		//cout<<"in send_request"<<endl;
		

		while(true)
		{
			//cout<<"in send_request"<<endl;

			
				switch(sending_state)
				{
				case free:
					{
						//if buffer is empty, do nothing
						if(req_write==req_read)
							break;

						//find free vc
						int k;
						for( k=0;k<vc_number;k++)
						{
							if (output_backward_interface.read().buffer_full[k] == false
								&& k == output_backward_interface.read().available_vc)
							{
								current_vc = k;
								break;
							}
						}
						//if no free vc, do nothing
						if(k>=vc_number)
							break;
						
						temp_packet=gen_packet[req_read];
						req_read=(req_read+1)%infinite;
			
						temp_packet.head=true;
						temp_packet.tail=false;
						
						
						wait(SC_ZERO_TIME);///simulate the line delay 
					
						router_forward_interface temp_data;
						temp_data.data_flit=temp_packet;
						temp_data.vc_id=current_vc;
						temp_data.ready=true;
						output_forward_interface.write(temp_data);

						//send_req_out_log<<sc_time_stamp()<<" 1 send packet "<<temp_packet.src<<"->"<<temp_packet.dest<<" sequence " <<temp_packet.packet_sequence<<endl;

						wait_packet_transmission=const_pkt_length-1;

						sending_state=core_sending_data;
						break;
					}

				case core_sending_data:
					{
						//if(buffer_full_in[current_vc].read()==true)
						//if buffer is full, skip, wait next cycle
						if(output_backward_interface.read().buffer_full[current_vc]==true)
						{
							break;
						}

						if(wait_packet_transmission>1)
						{
							temp_packet.head=false;
							temp_packet.tail=false;
							temp_packet.packet_sequence=temp_packet.packet_sequence+1000000;
							
							wait(SC_ZERO_TIME);///simulate the line delay 
						
							router_forward_interface temp_data;
							temp_data.data_flit=temp_packet;
							temp_data.vc_id=current_vc;
							temp_data.ready=true;
							output_forward_interface.write(temp_data);

						}
						if(wait_packet_transmission==1)
						{
							temp_packet.head=false;
							temp_packet.tail=true;

							temp_packet.packet_sequence=temp_packet.packet_sequence+1000000;
							//send_req_out_log<<sc_time_stamp()<<" 1.5 finish packet "<<temp_packet.src<<"->"<<temp_packet.dest<<" sequence " <<temp_packet.packet_sequence<<endl;
								
							wait(SC_ZERO_TIME);///simulate the line delay 
							
							router_forward_interface temp_data;
							temp_data.data_flit=temp_packet;
							temp_data.vc_id=current_vc;
							temp_data.ready=true;
							output_forward_interface.write(temp_data);
						}

						wait_packet_transmission--;

						if(wait_packet_transmission==0)
						{
							sending_state=free;
							clear_write_signal_time_remain=1;
						}

						break;
					}

				}//end switch
			
			wait();
			//clear write signal 
			wait(SC_ZERO_TIME);
			
				//if(clear_write_signal_time_remain[i]==1 )
				//{//take care here, we should make sure it is buffer_full=false
				//	clear_write_signal_time_remain[i]=0;
					//data_request[i].write(false);
					router_forward_interface temp_data;
					temp_data.ready=false;
					output_forward_interface.write(temp_data);

				//}
			
		
		}//end while
}


void electronic_processor::receive_packet()
{
		
		//cout<<sc_simulation_time()<<" in receive_packet"<<endl;
		flit_type temp_flit;
		

			if(input_forward_interface.read().ready==true)
			{
				
				temp_flit=input_forward_interface.read().data_flit;
				//send_req_out_log<<sc_time_stamp()<<" 2 receive packet "<<temp_flit.src<<"->"<<temp_flit.dest<<" sequence " <<temp_flit.packet_sequence<<endl;

				if(temp_flit.dest!=id)
				{
							cout<<"error in receiver"<<endl;
				}

				if(temp_flit.tail==true)
				{
						
						if(temp_flit.dest!=id)
						{
							cout<<"error in receiver"<<endl;
						}
						else
						{
							int qr;
							qr=int(sc_simulation_time());
							//send_req_out_log<<sc_time_stamp()<<"chip "<<i<<" receive packet "<<temp_flit.src<<"->"<<temp_flit.dest<<" sequence " <<temp_flit.packet_sequence<<endl;
							int true_sequence;
							true_sequence=temp_flit.packet_sequence - (const_pkt_length-1)*1000000;
							//send_req_out_log<<"-2 "<<temp_flit.packet_sequence<<" "<<temp_flit.src<<" "<<id<<" "<<qr<<endl;
							send_req_out_log<<"-2 "<<true_sequence<<" "<<temp_flit.src<<" "<<id<<" "<<qr<<endl;
							//temp_flit.packet_sequence-(pkt_size-1)*1000000 this is the true packet sequence
							//cout<<"receive something at "<<qr<<endl;
						}
	

						int temp=int (sc_time_stamp().to_double()/1000);//to ns
						temp=temp-temp_flit.generate_time;
						total_delay+=temp;
						total_packet++;
						total_power+=temp_flit.power+LocalConnectPower*temp_flit.packet_length*32;		

						
				}

			}
		
		
}

void electronic_processor::	initial()
//initial: Can NOLY BE called by outside because the output port is not bound yet!
	{
					
			router_forward_interface temp_data;
			temp_data.ready=false;
			output_forward_interface.write(temp_data);

			router_backward_interface temp_back_data;
			for(int i=0;i<vc_number;i++){
				temp_back_data.buffer_full[i]=false;
			}
			temp_back_data.available_vc=0;
			temp_back_data.updated_latency = 0;
			temp_back_data.updated_latency_flag = false;
			input_backward_interface.write(temp_back_data);

	
			
		

	}
  


#endif