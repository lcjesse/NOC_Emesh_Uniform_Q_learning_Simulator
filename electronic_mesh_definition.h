#ifndef electronic_mesh_definition
#define electronic_mesh_definition

#include "systemc.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;
const int MAX_LOOP_NUMBER_ALLOWED=50;

// mesh-based topology
const int NOC_WIDTH = 4;
const int NOC_HEIGHT = 4;
const int processor_no=NOC_WIDTH*NOC_HEIGHT;
const int core_no=processor_no;
#define X_change_road_threshold NOC_WIDTH/2+1
#define Y_change_road_threshold NOC_HEIGHT/2+1
const int port_no=5;//mesh
// definition of ports in a router
#define LEFT				0
#define RIGHT				1
#define UP					2
#define DOWN				3
#define TO_PROCESSOR		4
#define CLOCK_CYCLE			1
//#define AMS			0.1////////////////////////////////
const int max_link_delay=100;
int const_pkt_length=128;
const int flit_size=32;
int simulation_time=100000;
const int infinite=3000;//should not be too large!!!!, else exception will be caught


const int vc_number=2;
const int vc_depth=16;

const int routing_algorithm_delay=1;

double AMS;///////////////////////////////////////////
int routing_algo;//////////////////////////////////////Routing algorithm


int traffic_model=0;

const double XbarPower=0.07;
const double BufferPower=0.0015;
const double Dynamic_power=0.0015;
const double RouterControlPower=1.8;
const double LocalConnectPower=0.04;
const double GlobalConnectPower=0.62;
const double BusPower_unit=3.6/2;//10
//const int total_register_number=(tree_height-2)*tree_width*4*32*vc_depth*vc_number + tree_width*2*32*vc_depth*vc_number + tree_width*2*32*top_level_vc_depth*vc_number;



//enum processor_core_type{core_idle,core_waiting_ };

typedef struct _Core_Node {
		vector<int>  predecessor_list;	// each entry is the cpu id of necessary predecessor
		vector<int> initial_token_on_coming_link;
		vector<int> consumption_rate;	// each entry is the number of packets needed from the predecessor cpu
		vector<int> incoming_count;		// each entry is the count of incoming packets, the input data is ready if the count equals to the consumption rate

		vector<int> incoming_edge_list; //each entry is the id of incoming edge, which is connecting predecessor and this node
		vector<int> outcoming_edge_list; //each entry is the id of outcoming edge, which is connecting this node and successor
		
		vector<int> successor_list;		// each entry is the cpu id of a successor
		vector<int> production_rate;	// each entry is the number of packets to send to the successor	
		vector<int> outcoming_count;	//each entry is the count of outcoming packets need to be sent out
		vector< vector<int> > generate_time; //the first rank is the successor id, the second rank is the time of each generated packet 
		vector< vector<int> > delay;// the first rank is the predecessor id, the second rank is the delay of each packet
		vector< vector<double> > energy;
		vector< vector<double> > switching_capacity_consumed;//the first rank is the predecessor id, the seconed rank is the switching capcacity consumed in the network of each packet: that is related to each router 
		vector< vector<int> > encoutered_router;//the number of routers encountered in the path
		
		vector<double> local_start_loop_time;
		vector<double> local_end_loop_time;
		bool local_start_point;
		bool local_end_point;

		int loop;
		int run_time;
		int current_time;
		int assigned_task_id;
		int output_rate;

}Core_Node;



struct flit_type
{
       int src;
       int dest;
	   int vc_choice;/////////////////////////////////////////////////////////select the vc to avoid deadlock.

	   int edge_id;//the edge in the appilcation 


	   int packet_length; //in flits
	   int packet_sequence;
	   int generate_time;
	   int direction;
	   double power;

	   double switching_capacity_consumed;
	   int encountered_router_number;

	   int waiting_time;
	   

	   bool head;
	   bool tail;
	   int routing_delay;

       flit_type& operator=(const flit_type&);
       bool operator==(const flit_type&) const;
};

inline flit_type& flit_type::operator =(const flit_type &arg)
{
       src=arg.src;
       dest=arg.dest;
	   edge_id=arg.edge_id;
	   vc_choice = arg.vc_choice;

	   packet_length=arg.packet_length;
	   packet_sequence=arg.packet_sequence;
	   generate_time=arg.generate_time;
	   head=arg.head;
	   tail=arg.tail;
	   direction=arg.direction;
	   power=arg.power;
	   routing_delay=arg.routing_delay;
	   switching_capacity_consumed=arg.switching_capacity_consumed;
	   encountered_router_number=arg.encountered_router_number;

	   waiting_time=arg.waiting_time;
	   

       return (*this);
}

inline bool flit_type::operator ==(const flit_type &arg) const
{
	return (src==arg.src)&&(dest==arg.dest) &&(vc_choice==arg.vc_choice)&&
		(packet_length==arg.packet_length)&&(edge_id == arg.edge_id) &&
		(packet_sequence==arg.packet_sequence)&&(generate_time==arg.generate_time)&&
		(head==arg.head)&&(tail==arg.tail)&&(direction==arg.direction)&&(power==arg.power)&&
		(switching_capacity_consumed==arg.switching_capacity_consumed)&&
		(encountered_router_number==arg.encountered_router_number);
}


inline ostream&
operator<<(ostream& os, const flit_type& arg)
{
	os<<"src="<<arg.src<<" dest="<<arg.dest<<endl;
       return os;
}

extern  void sc_trace(sc_trace_file *tf, const flit_type& arg, const  std::string& name) 
{
       sc_trace(tf,arg.src,"src");//,name ".src");
       sc_trace(tf,arg.dest,"dest");//,name + ".dest");
	   sc_trace(tf,arg.packet_sequence,"seq");
}


struct Request_buffer
	{
		int src;
		int dest;
		bool wait_req_flag;
		bool wait_tear_flag;
	};

struct router_forward_interface
{
	bool ready;
	int vc_id;
	flit_type data_flit;
	router_forward_interface& operator=(const router_forward_interface&);
	bool operator==(const router_forward_interface&) const;
};

inline router_forward_interface& router_forward_interface::operator =(const router_forward_interface &arg)
{
       ready=arg.ready;
	   vc_id=arg.vc_id;
	   data_flit=arg.data_flit;

       return (*this);
}

inline bool router_forward_interface::operator ==(const router_forward_interface &arg) const
{
	return (ready==arg.ready) && (vc_id==arg.vc_id) && (data_flit==arg.data_flit);
}


inline ostream&
operator<<(ostream& os, const router_forward_interface& arg)
{
	os<<"ready= "<<arg.ready<<", Vc_id="<<arg.vc_id<<",data:"<<arg.data_flit<<endl;
       return os;
}
extern  void sc_trace(sc_trace_file *tf, const router_forward_interface& arg, const  std::string& name) 
{
	sc_trace(tf,arg.vc_id,"src");

}

struct router_backward_interface
{
	bool buffer_full[vc_number];
	int available_vc;
	int available_vc1;////////////////////////////////////////////////////////////////////////////////////////////////////another available vc
	int num_available_vc;///////////////////////////////////////////////////////////////////////////////////////////////number of available vc
	int updated_latency;   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Updated Q value
	bool updated_latency_flag;//////////////////////////////////////////////////////////////////////////////////////////////////////////////Signal to update Q value
	int updated_latency_dst;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////Return dst infomation that Q packet carrys

	router_backward_interface& operator=(const router_backward_interface&);
	bool operator==(const router_backward_interface&) const;
};

inline router_backward_interface& router_backward_interface::operator =(const router_backward_interface &arg)
{
       for(int i=0;i<vc_number;i++)
		   buffer_full[i]=arg.buffer_full[i];
	   available_vc=arg.available_vc;
	   available_vc1 = arg.available_vc1;
	   num_available_vc = arg.num_available_vc;
	   updated_latency = arg.updated_latency;///////////////////////////////////////////////////////////////////////////////////////////////////
	   updated_latency_flag = arg.updated_latency_flag;
	   updated_latency_dst = arg.updated_latency_dst;

       return (*this);
}

inline bool router_backward_interface::operator ==(const router_backward_interface &arg) const
{

	for(int i=0;i<vc_number;i++)
	{
		if(buffer_full[i]!=arg.buffer_full[i])
			return false;
	}
	return ((available_vc == arg.available_vc) && (available_vc1 == arg.available_vc1) && (num_available_vc == arg.num_available_vc) && (updated_latency == arg.updated_latency) && (updated_latency_flag == arg.updated_latency_flag) && (updated_latency_dst == arg.updated_latency_dst));
}////////////////////////////////////////////////////////////////////////////////////////////



inline ostream&
operator<<(ostream& os, const router_backward_interface& arg)
{
	os<<"available_vc="<<arg.available_vc<<endl;
       return os;
}
extern  void sc_trace(sc_trace_file *tf, const router_backward_interface& arg, const  std::string& name) 
{
	sc_trace(tf,arg.available_vc,"src");

}

class interchip_control_packet_type
{
public:

	bool request;
	bool tear_down;
	bool grant;
	bool fail;
	int src_chip;
	int dest_chip;
	int bus_id;
	

	interchip_control_packet_type& operator = (const interchip_control_packet_type& arg)
	{
		request=arg.request;
		tear_down=arg.tear_down;
		grant=arg.grant;
		fail=arg.fail;
		src_chip=arg.src_chip;
		dest_chip=arg.dest_chip;
		bus_id=arg.bus_id;
		return (*this);
	}
	 bool operator==(const interchip_control_packet_type& arg) const
	 {
		 return ((request==arg.request)&&(tear_down==arg.tear_down)&&(grant==arg.grant)&&(fail==arg.fail)&&(src_chip==arg.src_chip)&&(dest_chip==arg.dest_chip)&&(bus_id==arg.bus_id));
	 }
};
inline ostream& 
operator<<(ostream& os, const interchip_control_packet_type& arg)
{
	os<<"req="<<arg.request<<endl;
       return os;
}

extern  void sc_trace(sc_trace_file *tf, const interchip_control_packet_type& arg, const  std::string& name) 
{
       sc_trace(tf,arg.request,"req");//,name ".src");
	   sc_trace(tf,arg.grant,"grant");//,name + ".dest");
	   sc_trace(tf,arg.fail,"fail");
}


#endif