//
//  Created by Joseph Mark on 17/08/15.
//  Copyright (c) 2015 Joseph Mark. All rights reserved.
//

#ifndef COLLISIONS_H
#define COLLISIONS_H

#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <OpenGL/gl3.h>

namespace collisions {

class Collidable
{
private:
	int type_;

	bool isCircle_;
	float restitution_;
	float inertia_;

	/// CIRCLE
	float centrex_;
	float centrey_;
	uint radius_;

	/// POLYGON
	uint nvert_;
	float* vertx_;
	float* verty_;
	float* normx_;
	float* normy_;
	
public:
	
	int id;

	Collidable() : isCircle_(false) {}
	Collidable(float x, float y, uint radius) : isCircle_(true), centrex_(x), centrey_(y), radius_(radius) {}
	Collidable(uint nvert, float* vertx, float* verty);
	Collidable(float x, float y, float w, float h);
	void setVertices(uint nvert, float* vertx, float* verty);
	void setNormals(uint nvert, float* vertx, float* verty, float* normx, float* normy);
	void transformVertices(glm::vec2 position, float angle);
	int type() const { return type_; }
	void setType(int type) { type_ = type; }
	
public:
	bool isCircle() const { return isCircle_; }
	float centreX() const { return centrex_; }
	float centreY() const { return centrey_; }
	uint radius() const { return radius_; }
	uint nvert() const { return nvert_; }
	float* vertx() const { return vertx_; }
	float* verty() const { return verty_; }
	float* normx() const { return normx_; }
	float* normy() const { return normy_; }
	float restitution() const { return restitution_; }
	float inertia() const { return inertia_; }
};

class Quadtree
{
private:
	std::vector<Quadtree> quads_;
	float* vertx_;
	float* verty_;
	std::vector<Collidable*> items_;
public:
	Quadtree() {}
	Quadtree(float* vertx, float* verty, uint depth);
	float* vertx() const;
	float* verty() const;
	std::vector<Collidable*> items() const;
	std::vector<Quadtree> quads() const;
	void clear();
	void insert(Collidable* item);
	void possibleCollision(Collidable* item_a, Collidable* item_b) const;
	std::vector<Collidable*> checkCollisions() const;
};

void setResolve(void(*resolve)(Collidable*, Collidable*, int, glm::vec2));

}

#endif