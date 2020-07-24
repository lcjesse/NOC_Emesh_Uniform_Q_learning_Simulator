/*********************************************************/
// Sensor Network On Chip
//
// Author:	Weichen Liu
// Date:	20 Jun 2009
/*********************************************************/


#include "electronic_mesh_definition.h"
#include "electronic_mesh.h"
#include "global_data.h"
int sc_main(int argc, char* argv[]) {
	if (argc == 5){
		injection_rate = atof(argv[1]);
		simulation_time=atoi(argv[2]);
		routing_algo = atoi(argv[3]);///////////////////////////////////////////////////////////////////////////////////////////////////////////路由方式选择参数
		AMS = atof(argv[4]);
	}
	else {
		injection_rate = 0.1;
		simulation_time=1000000;
		routing_algo = 0;
		AMS = 1;
	}	
	//puts(filename);
	sc_clock clk("clk", CLOCK_CYCLE, SC_NS);
	NetworkOnChip *noc;
	noc = new NetworkOnChip("noc");
	noc->clk(clk);
	noc->init(clk);

	cout<<"\nStarting Simulation\n\n";
	sc_start(simulation_time, SC_NS);

//	sc_close_vcd_trace_file(tf);
	cout<<"\nTerminating Simulation\n\n";
	double total_latency=0;
	double total_throughput=0;
	double total_pkt_num=0;
	double average_latency=0;
	int qtable_sample[NOC_WIDTH * NOC_HEIGHT][NOC_WIDTH * NOC_HEIGHT][4];/////////////////////////////////////////////////////////////check qtable
	for (int i=0; i<NOC_WIDTH; i++) {
			for (int j=0; j<NOC_HEIGHT; j++){
				total_pkt_num=total_pkt_num+noc->processor[i][j].total_packet;
				total_latency=total_latency+noc->processor[i][j].total_delay;
				for (int p = 0; p < NOC_WIDTH * NOC_HEIGHT; p++)
				{
					for (int k = 0; k < 4; k++)
					{
						qtable_sample[i * NOC_HEIGHT + j][p][k] = noc->router[i][j].qtable[p][k];
					}
				}///////////////////////////////////////////////////////////////////////////////////////////////////check qtable
			}
	}
	average_latency=total_latency/total_pkt_num;
	total_throughput=total_pkt_num*const_pkt_length*flit_size/simulation_time;



	ofstream formatted_throughput_delay_log;
	ofstream throughput_delay_log;

	formatted_throughput_delay_log.open("q_throughput_delay.txt",ios::out|ios::app);//("odd_even_normal_vs_q_throughput_delay.txt", ios::out | ios::app);//("q1000000_512byte_8x4_uniform_formatted_2D_EMesh_throughput_delay.txt",ios::out|ios::app);

	formatted_throughput_delay_log << injection_rate << "	" << total_throughput << "	" << average_latency << "	" << routing_algo << " " << total_pkt_num << " " << AMS << " ";
	formatted_throughput_delay_log << "\n";
	//for (int p = 0; p < NOC_WIDTH; p++)
	//{
	//	for (int k = 0; k < NOC_HEIGHT; k++)
	//	{
	//		for (int i = 0; i < NOC_WIDTH * NOC_HEIGHT; i++)
	//		{
	//			for (int j = 0; j < 4; j++)
	//			{
	//				formatted_throughput_delay_log << qtable_sample[p * NOC_HEIGHT + k][i][j] << "    ";
	//			}
	//			formatted_throughput_delay_log << "\n";//////////////////////////////////////////////////////////////////////check qtable
	//		}
	//		formatted_throughput_delay_log << "\n";
	//	}
	//}
	for (int i = 0; i < NOC_WIDTH * NOC_HEIGHT; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			formatted_throughput_delay_log << qtable_sample[6][i][j] << "    ";
		}
		formatted_throughput_delay_log << "\n";//////////////////////////////////////////////////////////////////////check qtable
	}

	formatted_throughput_delay_log<<endl;
								

	formatted_throughput_delay_log.close();
	
	
	//noc->finalize();
	//cin.get();
	return 0;
}


