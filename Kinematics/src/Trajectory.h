/*
 * Trajectory.h
 *
 *  Created on: 27.07.2017
 *      Author: JochenAlt
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "basics/serializer.h"
#include "basics/types.h"
#include "basics/point.h"
#include "spatial.h"

#include <vector>
#include "setup.h"

class Trajectory : public Serializable  {
	public:
		friend ostream& operator<<(ostream&, const Trajectory&);

		Trajectory() {
			null();
		};

		Trajectory(const Trajectory & t) {
			path = t.path;
			generationNumber = t.generationNumber;
		};
		virtual ~Trajectory() {};


		void clear() {
			path.clear();
		}

		void add(const StampedPose& p) {
			path.push_back(p);
		}


		void null() {
			path.clear();
		}

		bool isNull() {
			return (path.size() == 0);
		}

		void operator= (const Trajectory& p) {
			path = p.path;
			generationNumber = p.generationNumber;
		}

		StampedPose& operator[] (int i) {
			return path[i];
		}

		unsigned int size() {
			return path.size();
		}

		virtual std::ostream& serialize(std::ostream &out) const;
		virtual std::istream& deserialize(std::istream &in, bool &ok);

		int getGenerationNumber() { return generationNumber; };
		void setGenerationNumber(int no) { generationNumber = no; };
private:
		vector<StampedPose> path;
		int generationNumber;
};

#endif
