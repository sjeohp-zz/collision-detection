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
		bool isCircle_;

		/// CIRCLE
		float centrex_;
		float centrey_;
		float radius_;

		/// POLYGON
		uint nvert_;
		float* vertx_;
		float* verty_;
		float* normx_;
		float* normy_;

		void updateNormals();
		long id_;
		long ind_;
	
	public:
		Collidable() : isCircle_(false) {}
		Collidable(float x, float y, float radius) : isCircle_(true), centrex_(x), centrey_(y), radius_(radius) {}
		Collidable(uint nvert, float* vertx, float* verty);
		Collidable(float x, float y, float w, float h);
		void rotateVertices(glm::vec2 position, float angle);
		long id() const { return id_; }
		void setId(long id) { id_ = id; }
		long ind() { return ind_; }
		void setInd(long ind) { ind_ = ind; }
		bool isCircle() const { return isCircle_; }
		float centreX() const { return centrex_; }
		float centreY() const { return centrey_; }
		float radius() const { return radius_; }
		uint nvert() const { return nvert_; }
		float* vertx() const { return vertx_; }
		float* verty() const { return verty_; }
		float* normx() const { return normx_; }
		float* normy() const { return normy_; }
	};

	class Quadtree
	{
	private:
		std::vector<Quadtree> quads_;
		float* vertx_;
		float* verty_;
		std::vector<Collidable*> items_;
	
	protected:
		float* vertx() const;
		float* verty() const;
		std::vector<Collidable*> items() const;
		std::vector<Quadtree> quads() const;
		void possibleCollision(Collidable* item_a, Collidable* item_b);
	
	public:
		Quadtree() {}
		Quadtree(float* vertx, float* verty, uint depth);
		void clear();
		void insert(Collidable* item);
		std::vector<Collidable*> checkCollisions();
	};

	void setResolve(void(*resolve)(Collidable*, Collidable*, int, glm::vec2));

}

#endif