#pragma once

#include "Hand.h"

#define WR(x) std::cout << x << std::flush
#define WRL(x) std::cout << x << std::endl

class HandGenerator{

public:
	HandGenerator(HandParameters hg){

		/*
		WRL("Init start");
		
		hg.CopyTo(&hp);

		bool hand_success = hand.Init(hp);
		if (!hand_success)
		{
			WRL("ERROR: Hand loading was not successfull, please check the parameters and/or your system");
			return;
		}
		WRL("hand OK");
		
		std::vector<int> temp;
		ComputePosition(&positions, temp, 0);
		WRL("positions OK");

		ComputeRotation(&rotations);
		WRL("rotations OK");
		*/
	}

	void run(){
		//printf("w:%d h:%d \n", hp.render_fbo_width, hp.render_fbo_height);
		//hand.Render(hp.render_fbo_width, hp.render_fbo_height, 0, 0, 0, false, 10000);

	}


private:
	Hand hand;
	HandParameters hp;

};