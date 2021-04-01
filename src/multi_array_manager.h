#ifndef MULTI_ARRAY_MANAGER_H
#define MULTI_ARRAY_MANAGER_H

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

template<class MultiArray>
class MultiArrayManager
{
	MultiArray msg;

	void create (std::vector<int> sizes, int dataOffset = 0) {
		int dimensions = sizes.size ();
		msg.layout.dim.resize (dimensions);
		msg.layout.data_offset = dataOffset;

		for (int i = dimensions-1; i >= 0; i--) {
			msg.layout.dim[i].size = sizes[i];
			if (i < dimensions - 1)
				msg.layout.dim[i].stride = sizes[i] * msg.layout.dim[i+1].stride;
			else
				msg.layout.dim[i].stride = sizes[i];
		}

		msg.data.resize (msg.layout.dim[0].stride + dataOffset);
	}

	int getIndex (std::vector<int> indexes) {
		int numDims = msg.layout.dim.size ();
		int index = msg.layout.data_offset;

		for (int i = 0; i < numDims; i++) {
			int currIndex = indexes[i];
			if (i == numDims - 1)
				index += currIndex;
			else
				index += currIndex * msg.layout.dim[i+1].stride;
		}

		return index;
	}

public:
	MultiArrayManager (std::vector<int> sizes, int dataOffset = 0) {
		create (sizes, dataOffset);
	}

	MultiArrayManager (const MultiArray &other) {
		msg = other;
	}

	double get (std::vector<int> indexes) {
		return msg.data[getIndex (indexes)];
	}

	void set (std::vector<int> indexes, double value) {
		msg.data[getIndex (indexes)] = value;
	}

	int size (int i) {
		return msg.layout.dim[i].size;
	}

	MultiArray getMsg () {
		return msg;
	}
};

using MultiArray32Manager = MultiArrayManager <std_msgs::Float32MultiArray>;
using MultiArray64Manager = MultiArrayManager <std_msgs::Float64MultiArray>;




#endif // MULTI_ARRAY_MANAGER_H
