/*
 * task_region.h
 *
 *  Created on: May 6, 2016
 *      Author: rdu
 */

#ifndef SRC_H2C_TASK_REGION_H_
#define SRC_H2C_TASK_REGION_H_

#include <cstdint>
#include <cstring>

namespace acel {

/// A type used to specify task regions. A region can be labeled from 0 to 31, which corresponds to "p0" to "p31".
class TaskRegion {
public:
	TaskRegion():region_label_(0),region_name_("p0"),bit_map_(0){};
	TaskRegion(uint8_t rid):region_label_(rid){
		region_name_ = "p" + std::to_string(rid);
		bit_map_ = 0x01 << rid;
	};
	~TaskRegion(){};

	const static uint8_t max_label_num = 32;

private:
	uint8_t region_label_;
	std::string region_name_;
	uint32_t bit_map_;

public:
	void SetRegionLabel(uint8_t new_label){
		if(new_label > max_label_num)
			new_label = max_label_num;

		region_label_ = new_label;
		region_name_ = "p" + std::to_string(new_label);
		bit_map_ = 0x01 << new_label;
	};

	uint32_t GetRegionLabel() const {
		return region_label_;
	}

	std::string GetRegionName() const {
		return region_name_;
	}

	uint32_t GetRegionBitMap() const {
		return bit_map_;
	}
};

}

#endif /* SRC_H2C_TASK_REGION_H_ */
