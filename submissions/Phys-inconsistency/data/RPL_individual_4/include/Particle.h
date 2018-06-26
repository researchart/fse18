/*
 * Particle.h
 *
 *  Created on: Dec 3, 2014
 *      Author: mafilipp
 *      Note: This class represent the particle used by the filter (for the case of pose estimation)
 */

#ifndef MAFILIPP_PARTICLE_FILTER_SRC_PARTICLE_H_
#define MAFILIPP_PARTICLE_FILTER_SRC_PARTICLE_H_

class Particle {
public:
	Particle();
	virtual ~Particle();

	// Setters and Getters
	double getTheta() const;
	void setTheta(double theta);
	double getX() const;
	void setX(double x);
	double getY() const;
	void setY(double y);

private:
	double x;
	double y;
	double theta;

};

#endif /* MAFILIPP_PARTICLE_FILTER_SRC_PARTICLE_H_ */
