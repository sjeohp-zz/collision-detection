//
//  Created by Joseph Mark on 17/08/15.
//  Copyright (c) 2015 Joseph Mark. All rights reserved.
//

#include "collisions.h"

namespace collisions {
	
void (*resolveCollision_)(Collidable*, Collidable*, int, glm::vec2);

void resolveCollision(Collidable* item_a, Collidable* item_b, int face_index, glm::vec2 support_point) 
{
	if (resolveCollision_ != NULL) {
		resolveCollision_(item_a, item_b, face_index, support_point);
	} else {
		std::cout << "Collision detected but unresolved; function pointer was not set." << std::endl;
	}
}

void setResolve(void(*resolve)(Collidable*, Collidable*, int, glm::vec2))
{
	resolveCollision_ = resolve;
}

/// COLLIDABLE

Collidable::Collidable(uint nvert, float* vertx, float* verty) 
: isCircle_(false), nvert_(nvert), vertx_(vertx), verty_(verty) 
{	
	normx_ = (float*)calloc(nvert_, sizeof(float));
	normy_ = (float*)calloc(nvert_, sizeof(float));

	setNormals(nvert_, vertx_, verty_, normx_, normy_);
}

Collidable::Collidable(float x, float y, float w, float h)
: isCircle_(false) 
{	
    float* vertx = (float*)calloc(4, sizeof(float));
	float* verty = (float*)calloc(4, sizeof(float));

    vertx[0] = x;
    verty[0] = y;
    vertx[1] = x + w;
    verty[1] = y;
    vertx[2] = x + w;
    verty[2] = y + h;
    vertx[3] = x;
    verty[3] = y + h;

    nvert_ = 4;
    vertx_ = vertx;
    verty_ = verty;

	normx_ = (float*)calloc(nvert_, sizeof(float));
	normy_ = (float*)calloc(nvert_, sizeof(float));

	setNormals(nvert_, vertx_, verty_, normx_, normy_);
}

void Collidable::setVertices(uint nvert, float* vertx, float* verty)
{
    nvert_ = nvert;
    vertx_ = vertx;
    verty_ = verty;
	for (int i = 0; i < nvert-1; ++i) {
        normx_[i] = verty[i+1] - verty[i];
        normy_[i] = -vertx[i+1] + vertx[i];
	}
    normx_[nvert-1] = verty[0] - verty[nvert-1];
    normy_[nvert-1] = -vertx[0] + vertx[nvert-1];
}

void Collidable::setNormals(uint nvert, float* vertx, float* verty, float* normx, float* normy) {
	for (int i = 0; i < nvert-1; ++i) {
        normx[i] = verty[i+1] - verty[i];
        normy[i] = -vertx[i+1] + vertx[i];
	}
    normx[nvert-1] = verty[0] - verty[nvert-1];
    normy[nvert-1] = -vertx[0] + vertx[nvert-1];
}

// Translates and rotates vertices and normals
void Collidable::transformVertices(glm::vec2 position, float angle)
{
    if (isCircle()) { return; }
    float r = angle;
    float sinr = sinf(r);
    float cosr = cosf(r);
    float x = position.x;
    float y = position.y;
    float a;
    float vx[nvert_];
    float vy[nvert_];
    for (int i = 0; i < nvert_; ++i){
        a = glm::length(glm::distance(glm::vec2(vertx_[i], verty_[i]), position));
        vx[i] = -cosr * a + x;
        vy[i] = -sinr * a + y;
    }
    setVertices(nvert_, vx, vy);
}

glm::vec2 supportPoint(Collidable* item, glm::vec2 dir) 
{
    float best_proj = -FLT_MAX;
    glm::vec2 best_vert;

    for (int i = 0; i < item->nvert(); ++i) {
        glm::vec2 vert = glm::vec2(item->vertx()[i], item->verty()[i]);
        float proj = glm::dot(vert, dir);
	        
        if (proj > best_proj) {
            best_vert = vert;
            best_proj = proj;
        }
    }
    return best_vert;
}

float penetration(Collidable* item_a, Collidable* item_b, uint* face_index, glm::vec2* support_point)
{
    float best_d = -FLT_MAX;
    uint best_i;
    glm::vec2 best_s;

    for (int i = 0; i < item_a->nvert(); ++i) {
        // Retrieve a face normal from A
        glm::vec2 n = glm::vec2(item_a->normx()[i], item_a->normy()[i]);
	        
        // Retrieve support point from B along -n
        glm::vec2 s = supportPoint(item_b, -n);
    
        // Retrieve vertex on face from A, transform into
        // B's model space
        glm::vec2 v = glm::vec2(item_a->vertx()[i], item_a->verty()[i]);

        // Compute penetration distance (in B's model space)
        float d = glm::dot(n, s - v);
    
        // Store greatest distance
        if (d > best_d) {
            best_d = d;
            best_i = i;
            best_s = s;
        }
    }
    *face_index = best_i;
    *support_point = best_s;

    return best_d;
}

/// QUADTREE

#define CLOCKWISE(A , B) A.y * B.x < A.x * B.y ? YES : NO

class LineSegment {
public:
    LineSegment(float vertxA, float vertyA, float vertxB, float vertyB)
    : ax(vertxA), ay(vertyA), bx(vertxB), by(vertyB)
    {}
    const float ax;
    const float ay;
    const float bx;
    const float by;
};

bool poly_contains_pnt(int nvert, float *vertx, float *verty, float pntx, float pnty);
bool poly_contains_poly(int nverta, float* ax, float* ay, int nvertb, float* bx, float* by);
float sqrf(float x);
float dist_sqrdf(float vx, float vy, float wx, float wy);
float dist_line_point(LineSegment line, float px, float py);
float dist_line_point(LineSegment line, float px, float py, glm::vec2* closestPnt);
uint dist_poly_circ(int nvert, float* vertx, float* verty, float circx, float circy);
uint dist_poly_circ(int nvert, float* vertx, float* verty, float circx, float circy, glm::vec2* closestPnt, uint* faceIndex);
uint dist_circ_circ(float ax, float ay, float bx, float by);

bool poly_contains_pnt(int nvert, float *vertx, float *verty, float pntx, float pnty)
{
    int i, j, c = false;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>pnty) != (verty[j]>pnty)) &&
            (pntx < (vertx[j]-vertx[i]) * (pnty-verty[i]) / (verty[j]-verty[i]) + vertx[i]) ){
            c = !c;
        }
    }
    return c;
}

bool poly_contains_poly(int nverta, float* ax, float* ay, int nvertb, float* bx, float* by)
{
    bool c = true;
    for (int i = 0; i < nvertb; ++i){
        if (!poly_contains_pnt(nverta, ax, ay, bx[i], by[i])){
            c = false;
        }
    }
    return c;
}

float sqrf(float x) { return x * x; }

float dist_sqrdf(float vx, float vy, float wx, float wy)
{
    return sqrf(vx - wx) + sqrf(vy - wy);
}

float dist_line_point(LineSegment line, float px, float py)
{
    float length_squared = dist_sqrdf(line.ax, line.ay, line.bx, line.by);
    if (length_squared == 0) return dist_sqrdf(px, py, line.ax, line.ay);
    float t = ((px - line.ax) * (line.bx - line.ax) + (py - line.ay) * (line.by - line.ay)) / length_squared;
    if (t < 0) {
        return sqrtf(dist_sqrdf(px, py, line.ax, line.ay));
    }
    if (t > 1){
        return sqrtf(dist_sqrdf(px, py, line.bx, line.by));
    }
    float cx = line.ax + t * (line.bx - line.ax);
    float cy = line.ay + t * (line.by - line.ay);
    return sqrtf(dist_sqrdf(px, py, cx, cy));
}

float dist_line_point(LineSegment line, float px, float py, glm::vec2* closest_point)
{
    float length_squared = dist_sqrdf(line.ax, line.ay, line.bx, line.by);
    if (length_squared == 0) return dist_sqrdf(px, py, line.ax, line.ay);
    float t = ((px - line.ax) * (line.bx - line.ax) + (py - line.ay) * (line.by - line.ay)) / length_squared;
    if (t < 0) {
        *closest_point = glm::vec2(line.ax, line.ay);
        return sqrtf(dist_sqrdf(px, py, line.ax, line.ay));
    }
    if (t > 1){
        *closest_point = glm::vec2(line.bx, line.by);
        return sqrtf(dist_sqrdf(px, py, line.bx, line.by));
    }
    float cx = line.ax + t * (line.bx - line.ax);
    float cy = line.ay + t * (line.by - line.ay);
    *closest_point = glm::vec2(cx, cy);
    return sqrtf(dist_sqrdf(px, py, cx, cy));
}

uint dist_poly_circ(uint nvert, float* vertx, float* verty, float circx, float circy)
{
    uint distance = -1;
    uint temp;
    for (int i = 0; i < nvert - 1; ++i){
        temp = dist_line_point(LineSegment(vertx[i], verty[i], vertx[i+1], verty[i+1]), circx, circy);
        if (temp < distance){
            distance = temp;
        }
    }
    temp = dist_line_point(LineSegment(vertx[nvert-1], verty[nvert-1], vertx[0], verty[0]), circx, circy);
    if (temp < distance){
        distance = temp;
    }
    return distance;
}

uint dist_poly_circ(uint nvert, float* vertx, float* verty, float circx, float circy, glm::vec2* closest_point, uint* face_index)
{
    uint distance = -1;
    uint temp;
    for (int i = 0; i < nvert - 1; ++i){
        glm::vec2 c = glm::vec2(0, 0);
        temp = dist_line_point(LineSegment(vertx[i], verty[i], vertx[i+1], verty[i+1]), circx, circy, &c);
        if (temp < distance){
            distance = temp;
            *closest_point = c;
            *face_index = i;
        }
    }
    glm::vec2 c = glm::vec2(0, 0);
    temp = dist_line_point(LineSegment(vertx[nvert-1], verty[nvert-1], vertx[0], verty[0]), circx, circy, &c);
    if (temp < distance){
        distance = temp;
        *closest_point = c;
        *face_index = nvert - 1;
    }
    return distance;
}

uint dist_circ_circ(float ax, float ay, float bx, float by)
{
    return sqrtf((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

Quadtree::Quadtree(float* vertx, float* verty, uint depth)
: vertx_(vertx), verty_(verty)
{        
    if (depth > 0){			
		quads_ = std::vector<Quadtree>(4);
							
        float* sub_vert = (float*)calloc(32, sizeof(float));
	
        sub_vert[0] = vertx[0];
        sub_vert[16] = verty[0];
        sub_vert[1] = sub_vert[0];
        sub_vert[17] = (verty[1] - verty[0]) / 2.0;
        sub_vert[2] = (vertx[3] - vertx[2]) / 2.0;
        sub_vert[18] = sub_vert[17];
        sub_vert[3] = sub_vert[2];
        sub_vert[19] = sub_vert[16];
	
        quads_[0] = Quadtree(sub_vert, &sub_vert[16], depth-1);
	
        sub_vert[4] = sub_vert[1];
        sub_vert[20] = sub_vert[17];
        sub_vert[5] = sub_vert[4];
        sub_vert[21] = verty[1];
        sub_vert[6] = sub_vert[2];
        sub_vert[22] = verty[2];
        sub_vert[7] = sub_vert[2];
        sub_vert[23] = sub_vert[20];
	
        quads_[1] = Quadtree(&sub_vert[4], &sub_vert[20], depth-1);
	
        sub_vert[8] = sub_vert[7];
        sub_vert[24] = sub_vert[23];
        sub_vert[9] = sub_vert[6];
        sub_vert[25] = sub_vert[22];
        sub_vert[10] = vertx[2];
        sub_vert[26] = sub_vert[25];
        sub_vert[11] = sub_vert[10];
        sub_vert[27] = sub_vert[24];
	
        quads_[2] = Quadtree(&sub_vert[8], &sub_vert[24], depth-1);
	
        sub_vert[12] = sub_vert[3];
        sub_vert[28] = sub_vert[19];
        sub_vert[13] = sub_vert[2];
        sub_vert[29] = sub_vert[18];
        sub_vert[14] = sub_vert[11];
        sub_vert[30] = sub_vert[27];
        sub_vert[15] = vertx[3];
        sub_vert[31] = verty[3];
	
        quads_[3] = Quadtree(&sub_vert[12], &sub_vert[28], depth-1);			
    }
}

float* Quadtree::vertx() const
{
    return vertx_;
}

float* Quadtree::verty() const
{
    return verty_;
}

std::vector<Collidable*> Quadtree::items() const
{
    return items_;
}

std::vector<Quadtree> Quadtree::quads() const
{
    return quads_;
}

void Quadtree::clear() 
{
	if (!quads_.empty()) {
		for (int i = 0; i < 4; ++i) {
			quads_[i].clear();
		}
	}
	items_.clear();
	items_.shrink_to_fit();
}

void Quadtree::insert(Collidable* item)
{
    if (item->isCircle()){
		if (!quads_.empty()) {  
            for (int i = 0; i < 4; ++i){
                if (poly_contains_pnt((uint)4, quads_[i].vertx(), quads_[i].verty(), item->centreX(), item->centreY())
                    &&
                    dist_poly_circ((uint)4, quads_[i].vertx(), quads_[i].verty(), item->centreX(), item->centreY()) < item->radius())
                {
                    quads_[i].insert(item);
                    return;
                }
            }
        } else if (poly_contains_pnt((uint)4, vertx(), verty(), item->centreX(), item->centreY())
            &&
            dist_poly_circ((uint)4, vertx(), verty(), item->centreX(), item->centreY()) < item->radius())
        {
            items_.push_back(item);
            return;
		}
    } else {
		if (!quads_.empty()) {  
	        for (int i = 0; i < 4; ++i){
	            if (poly_contains_poly((uint)4, quads_[i].vertx(), quads_[i].verty(), item->nvert(), item->vertx(), item->verty())){
	                quads_[i].insert(item);
	                return;
	            }
	        }
        } else if (poly_contains_poly((uint)4, vertx(), verty(), item->nvert(), item->vertx(), item->verty())) {
            items_.push_back(item);
            return;
        }
	}
}

void Quadtree::possibleCollision(Collidable* item_a, Collidable* item_b) const
{
    if (item_a->isCircle() && item_b->isCircle()){
        if (dist_circ_circ(item_a->centreX(), item_a->centreY(), item_b->centreX(), item_b->centreY()) < item_a->radius() + item_b->radius()){
            resolveCollision(item_a, item_b, -1, glm::vec2(0));
        }
    } else if (item_a->isCircle()){
        glm::vec2 support_point;
        uint face_index;
        if (dist_poly_circ(item_b->nvert(), item_b->vertx(), item_b->verty(), item_a->centreX(), item_a->centreY(), &support_point, &face_index) <= item_a->radius()){
            resolveCollision(item_b, item_a, face_index, support_point);
        }
    } else if (item_b->isCircle()){
        glm::vec2 support_point;
        uint face_index;
        if (dist_poly_circ(item_a->nvert(), item_a->vertx(), item_a->verty(), item_b->centreX(), item_b->centreY(), &support_point, &face_index) <= item_b->radius()){
            resolveCollision(item_a, item_b, face_index, support_point);
        }
    } else {			
        uint face_index_a = USHRT_MAX;
        uint face_index_b = USHRT_MAX;
        glm::vec2 support_point_a;
        glm::vec2 support_point_b;
        float d1 = penetration(item_a, item_b, &face_index_a, &support_point_a);
        float d2 = penetration(item_b, item_a, &face_index_b, &support_point_b);
        if (d1 < 0 || d2 < 0) {
            if (d1 > d2) {
                resolveCollision(item_a, item_b, face_index_a, support_point_a);
            } else {
                resolveCollision(item_b, item_a, face_index_b, support_point_b);
            }
        }
    }
}

std::vector<Collidable*> Quadtree::checkCollisions() const
{
    std::vector<Collidable*> all_items;
    if (!quads_.empty()){
        std::vector<Collidable*> quad_items;
        quad_items = quads_[0].checkCollisions();
        for (std::vector<Collidable*>::iterator it = quad_items.begin(); it != quad_items.end(); ++it){
            all_items.push_back(*it);
            for (std::vector<Collidable*>::iterator ita = items().begin(); ita != items().end(); ++ita){
                possibleCollision(*ita, *it);
            }
        }
        quad_items = quads_[1].checkCollisions();
        for (std::vector<Collidable*>::iterator it = quad_items.begin(); it != quad_items.end(); ++it){
            all_items.push_back(*it);
            for (std::vector<Collidable*>::iterator ita = items().begin(); ita != items().end(); ++ita){
                possibleCollision(*ita, *it);
            }
        }
        quad_items = quads_[2].checkCollisions();
        for (std::vector<Collidable*>::iterator it = quad_items.begin(); it != quad_items.end(); ++it){
            all_items.push_back(*it);
            for (std::vector<Collidable*>::iterator ita = items().begin(); ita != items().end(); ++ita){
                possibleCollision(*ita, *it);
            }
        }
        quad_items = quads_[3].checkCollisions();
        for (std::vector<Collidable*>::iterator it = quad_items.begin(); it != quad_items.end(); ++it){
            all_items.push_back(*it);
            for (std::vector<Collidable*>::iterator ita = items().begin(); ita != items().end(); ++ita){
                possibleCollision(*ita, *it);
            }
        }
    }
    u_long s = items().size();
    for (int i = 0; i < s; ++i){
        for (int j = i+1; j < s; ++j){
            possibleCollision(items()[i], items()[j]);
            all_items.push_back(items()[i]);
            all_items.push_back(items()[j]);
        }
    }		
    return all_items;
}
}