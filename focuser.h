//focuser.h
//an interface class that represents a focusing component
//lensadapter or zstage

#pragma once

#include "paramHandler.h"

class Focuser{

	public:

		virtual ~Focuser() = 0;

		virtual double setFocus(int, double) = 0;

		virtual void setParams(int, ParamHandler*, const char*, int) = 0;

		virtual double getMinFocus(int) = 0;

		virtual double getMaxFocus(int) =0;
	
		virtual double getFocusStep(int)=0;

		virtual int addLens(const char*, int)=0;

		double findBestFocus(std::vector<std::vector<blob> >, int, int);

	private:
		
		bool blobContains(blob, std::vector<blob> &);

};
