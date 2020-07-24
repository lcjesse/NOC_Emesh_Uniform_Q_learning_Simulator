#ifndef NOC_MESH
#define NOC_MESH

#include "electronic_mesh_definition.h"
#include "router.h"
#include "electronic_processor.h"
#include "global_data.h"

//////////////////////////////////////////////////////////
//
//	Definition of signals
//
//    				o
//    				|^
//    				||
//    				||
//   connectToLower || connectToUpper
//    				||
//    				||
//    				v|    connectToRight
//    				o ----------------------> o
//     				 <----------------------
//     				      connectToLeft
//
//////////////////////////////////////////////////////////


class NetworkOnChip : public sc_module
{
public:
	SC_HAS_PROCESS(NetworkOnChip);

	NetworkOnChip(sc_module_name name=sc_gen_unique_name("NetworkOnChip")) : sc_module(name) {

		construct();
	}


	void init(sc_clock &global_clk) {
		for (int i=0; i<NOC_WIDTH; i++) {
			for (int j=0; j<NOC_HEIGHT; j++){
				router[i][j].clk(global_clk);
				processor[i][j].clk(global_clk);
				//processor[i][j].load_traffic();
			}
		}
		
	}

	

	void construct() {
		int i,j;
	//	cout << "@ " << sc_time_stamp() << " In construct() of NetworkOnChip" <<endl;

		// set the id and pos
		// initialize the routers

		for (i=0; i<NOC_WIDTH; i++) {
			for (j=0; j<NOC_HEIGHT; j++){
				router[i][j].setid(j*NOC_WIDTH+i);
				processor[i][j].setid(j*NOC_WIDTH+i);

				router[i][j].setpos(i,j);
				processor[i][j].setpos(i,j);

				//router[i][j].init();
			//	processor[i][j].cache_size = CACHE_SIZE;
			}
		}

		// connect the routers
		for (i=0; i<NOC_WIDTH-1; i++) {
			for (j=0; j<NOC_HEIGHT; j++){
				router[i][j].output_forward_interface[router[i][j].get_right()](connectToRight[i][j]);
				router[i+1][j].input_forward_interface[router[i+1][j].get_left()](connectToRight[i][j]);

				router[i][j].input_forward_interface[router[i][j].get_right()](connectToLeft[i][j]);
				router[i+1][j].output_forward_interface[router[i+1][j].get_left()](connectToLeft[i][j]);

				router[i][j].output_backward_interface[router[i][j].get_right()](fullConnectToLeft[i][j]);
				router[i+1][j].input_backward_interface[router[i+1][j].get_left()](fullConnectToLeft[i][j]);

				router[i][j].input_backward_interface[router[i][j].get_right()](fullConnectToRight[i][j]);
				router[i+1][j].output_backward_interface[router[i+1][j].get_left()](fullConnectToRight[i][j]);

				//cout << "Connecting router("<<i+1<<","<<j<<") to router("<<i<<","<<j<<") by toLeft("<<i<<","<<j<<")"<<endl;
			}
		}
		for (j=0; j<NOC_HEIGHT; j++) {
				router[0][j].output_forward_interface[router[0][j].get_left()](connectToRight[NOC_WIDTH-1][j]);
				router[NOC_WIDTH-1][j].input_forward_interface[router[NOC_WIDTH-1][j].get_right()](connectToRight[NOC_WIDTH-1][j]);

				router[0][j].input_forward_interface[router[0][j].get_left()](connectToLeft[NOC_WIDTH-1][j]);
				router[NOC_WIDTH-1][j].output_forward_interface[router[NOC_WIDTH-1][j].get_right()](connectToLeft[NOC_WIDTH-1][j]);

				router[0][j].output_backward_interface[router[0][j].get_left()](fullConnectToLeft[NOC_WIDTH-1][j]);
				router[NOC_WIDTH-1][j].input_backward_interface[router[NOC_WIDTH-1][j].get_right()](fullConnectToLeft[NOC_WIDTH-1][j]);

				router[0][j].input_backward_interface[router[0][j].get_left()](fullConnectToRight[NOC_WIDTH-1][j]);
				router[NOC_WIDTH-1][j].output_backward_interface[router[NOC_WIDTH-1][j].get_right()](fullConnectToRight[NOC_WIDTH-1][j]);
				
						
		}		
		
		
		for (i=0; i<NOC_WIDTH; i++) {
			for (j=0; j<NOC_HEIGHT-1; j++){
				router[i][j].output_forward_interface[router[i][j].get_up()](connectToUpper[i][j]);
				router[i][j+1].input_forward_interface[router[i][j+1].get_down()](connectToUpper[i][j]);

				router[i][j].output_backward_interface[router[i][j].get_up()](fullConnectToLower[i][j]);
				router[i][j+1].input_backward_interface[router[i][j+1].get_down()](fullConnectToLower[i][j]);

				router[i][j].input_backward_interface[router[i][j].get_up()](fullConnectToUpper[i][j]);
				router[i][j+1].output_backward_interface[router[i][j+1].get_down()](fullConnectToUpper[i][j]);

				router[i][j].input_forward_interface[router[i][j].get_up()](connectToLower[i][j]);
				router[i][j+1].output_forward_interface[router[i][j+1].get_down()](connectToLower[i][j]);

				
				
			//	cout << "Connecting router("<<i+1<<","<<j<<") to router("<<i<<","<<j<<") by toLower("<<i<<","<<j<<")"<<endl;
			}
		}
		for (i=0; i<NOC_WIDTH; i++) {
				router[i][0].output_forward_interface[router[i][0].get_down()](connectToUpper[i][NOC_HEIGHT-1]);
				router[i][NOC_HEIGHT-1].input_forward_interface[router[i][NOC_HEIGHT-1].get_up()](connectToUpper[i][NOC_HEIGHT-1]);

				router[i][0].output_backward_interface[router[i][0].get_down()](fullConnectToLower[i][NOC_HEIGHT-1]);
				router[i][NOC_HEIGHT-1].input_backward_interface[router[i][NOC_HEIGHT-1].get_up()](fullConnectToLower[i][NOC_HEIGHT-1]);

				router[i][0].input_backward_interface[router[i][0].get_down()](fullConnectToUpper[i][NOC_HEIGHT-1]);
				router[i][NOC_HEIGHT-1].output_backward_interface[router[i][NOC_HEIGHT-1].get_up()](fullConnectToUpper[i][NOC_HEIGHT-1]);

				router[i][0].input_forward_interface[router[i][0].get_down()](connectToLower[i][NOC_HEIGHT-1]);
				router[i][NOC_HEIGHT-1].output_forward_interface[router[i][NOC_HEIGHT-1].get_up()](connectToLower[i][NOC_HEIGHT-1]);

		}		
		
		for (i=0; i<NOC_WIDTH; i++) {
			for (j=0; j<NOC_HEIGHT; j++){		
				router[i][j].output_forward_interface[router[i][j].get_toproc()](connectToProc[i][j]);
				processor[i][j].input_forward_interface(connectToProc[i][j]);

				router[i][j].input_forward_interface[router[i][j].get_toproc()](connectFromProc[i][j]);
				processor[i][j].output_forward_interface(connectFromProc[i][j]);

				router[i][j].input_backward_interface[router[i][j].get_toproc()](fullConnectToProc[i][j]);
				processor[i][j].output_backward_interface(fullConnectToProc[i][j]);

				router[i][j].output_backward_interface[router[i][j].get_toproc()](fullConnectFromProc[i][j]);
				processor[i][j].input_backward_interface(fullConnectFromProc[i][j]);
				
				//	cout << "Connecting proc("<<i<<","<<j<<") to router("<<i<<","<<j<<") by fromProc("<<i<<","<<j<<")"<<endl;
			}
		}
		for (i=0; i<NOC_WIDTH; i++) {
			for (j=0; j<NOC_HEIGHT; j++){	
				processor[i][j].initial();
			}
		}
		
	}

public:	
	sc_in<bool>				clk;
//	Controller				controller;
	Router					router[NOC_WIDTH][NOC_HEIGHT];
	electronic_processor	processor[NOC_WIDTH][NOC_HEIGHT];
//	Memory					memory[NUM_MEMORY]; // memories are numbered by left 0, right 1, up 2, down 3
	
	// signals for data
	sc_signal<router_forward_interface>	connectToRight[NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_forward_interface>	connectToLeft [NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_forward_interface>	connectToUpper[NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_forward_interface>	connectToLower[NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_forward_interface>	connectToProc [NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_forward_interface>	connectFromProc[NOC_WIDTH][NOC_HEIGHT];
//	sc_signal<router_forward_interface>	connectToController;
//	sc_signal<router_forward_interface>	connectFromController;
//	sc_signal<router_forward_interface>	connectToMemory[NUM_MEMORY];
//	sc_signal<router_forward_interface>	connectFromMemory[NUM_MEMORY];

	// signals for data ready
	//sc_signal<bool>			readyConnectToRight[NOC_WIDTH][NOC_HEIGHT];
	//sc_signal<bool>			readyConnectToLeft [NOC_WIDTH][NOC_HEIGHT];
	//sc_signal<bool>			readyConnectToUpper[NOC_WIDTH][NOC_HEIGHT];
	//sc_signal<bool>			readyConnectToLower[NOC_WIDTH][NOC_HEIGHT];
	//sc_signal<bool>			readyConnectToProc [NOC_WIDTH][NOC_HEIGHT];
	//sc_signal<bool>			readyConnectFromProc[NOC_WIDTH][NOC_HEIGHT];
//	sc_signal<bool>			readyConnectToController;
//	sc_signal<bool>			readyConnectFromController;
//	sc_signal<bool>			readyConnectToMemory[NUM_MEMORY];
//	sc_signal<bool>			readyConnectFromMemory[NUM_MEMORY];

	// signals for buffer full
	sc_signal<router_backward_interface>			fullConnectToRight[NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_backward_interface>			fullConnectToLeft [NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_backward_interface>			fullConnectToUpper[NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_backward_interface>			fullConnectToLower[NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_backward_interface>			fullConnectToProc [NOC_WIDTH][NOC_HEIGHT];
	sc_signal<router_backward_interface>			fullConnectFromProc[NOC_WIDTH][NOC_HEIGHT];
//	sc_signal<bool>			fullConnectToController;
//	sc_signal<bool>			fullConnectFromController;
//	sc_signal<bool>			fullConnectToMemory[NUM_MEMORY];
//	sc_signal<bool>			fullConnectFromMemory[NUM_MEMORY];
};


#endif


