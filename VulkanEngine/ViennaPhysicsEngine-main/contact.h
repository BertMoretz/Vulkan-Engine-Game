#ifndef CONTACT_H
#define CONTACT_H

#include <iostream>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

#include "hash.h"
#include <random>
#include <set>

#include "collider.h"
#include "sat.h"

using namespace glm;

using vec3pair = std::pair<vec3,vec3>;


namespace vpe {

    //a contact stores a contact point between two objects
    //normal is the contact normal pointing away from obj2
    struct contact {
        Polytope *obj1;
        Polytope *obj2;
        vec3 pos;
        vec3 normal;

        bool operator <(const contact& c) const ;//need this for std::set
    };

}



namespace vpe {

bool contact::operator <(const contact& c) const {
    float x = (float) pos.x;
    float y = (float) pos.y;
    float z = (float) pos.z;
    return x < c.pos.x || y < c.pos.y || z < c.pos.z;
}

bool check_collision( Vertex &vertex, Face &face, vec3 &dir ) {
    vec3 p = vertex.pointW();

    vec3 converted(face.pluecker());
    float distance_point_plane = std::abs(dot(converted,p) + face.pluecker().w) / length(face.pluecker());
    return distance_point_plane < EPS && face.inside_cell(p);
}


void process_vertex_face_contact( Vertex &vertex, Face &face, std::set<contact> & contacts) {
    vec3 dir;
    if( check_collision( vertex, face, dir ) ) {
        contacts.insert( { vertex.m_polytope, face.m_polytope, vertex.pointW(), face.get_face_normal() }  );
    }
}

void process_edge_edge_contact( Face &face1, Line &edge1, Face &face2, Line &edge2, std::set<contact> & contacts) {
    vec3 v1 = edge1.pluecker().dir;
    vec3 m1 = edge1.pluecker().moment;
    vec3 v2 = edge2.pluecker().dir;
    vec3 m2 = edge2.pluecker().moment;
    float result_distance = std::abs(dot(v1,m2) + dot(v2,m1)) / length(cross(v1,v2));

    if( result_distance < EPS ) {
        vec3  v = edge2.pluecker().dir;
        vec3  m = edge2.pluecker().moment;
        
        vec3  n = face1.pluecker();
        float d = face1.pluecker().w;
        float po = -1.0f * dot(n,v);
        vec3 cross_vector = cross(m,n) + d*v;
        pluecker_point point = vec4{cross_vector, po};
        if( std::abs(po) < EPS ) return; // if 0 then both are parallel
        vec3 p = point;
        if( edge1.in_segment(p) && edge2.in_segment(p) ) {
            contacts.insert( { face1.m_polytope, face2.m_polytope, p, face2.get_face_normal() }  );
        }
    }
}

//test a pair of faces against each other.
//collide each vertex from one face with the other face
//collide each edge from one face with all edges from the other face
void process_face_face_contact(    Polytope &obj1, Polytope &obj2, vec3 &dir
                                ,  int f1, int f2, std::set<contact> & contacts ) {
    
    Face &face1 = obj1.m_faces[f1];
    Face &face2 = obj1.m_faces[f2];
    for( int v1 : face1.m_data->m_vertices ) {      //go through all vertices of face 1
        if( sat( obj1.m_vertices[v1], face2, dir) ) {
            process_vertex_face_contact( face1.m_polytope->m_vertices[v1], face2, contacts );
        }
    }

    for( int v2 : face2.m_data->m_vertices ) {      //go through all vertices of face 2
        if( sat( obj2.m_vertices[v2], face1, dir) ) {
            process_vertex_face_contact( face2.m_polytope->m_vertices[v2], face1, contacts );
        }
    }

    std::vector<Line> edges1;
    std::vector<Line> edges2;
    face1.get_edges( edges1 );
    face2.get_edges( edges2 );

    for( auto& edge1 : edges1 ) {      //go through all edge pairs
        for( auto& edge2 : edges2 ) {
            if( sat( edge1, edge2, dir) ) {
                process_edge_edge_contact( face1, edge1, face2, edge2, contacts );
            }
        }
    }
}


//find a list of face-pairs that touch each other
//process these pairs by colliding a face agains vertices and edges of the other face
void process_face_obj_contacts(     Polytope &obj1, Polytope &obj2, vec3 &dir
                                ,   std::vector<int>& obj1_faces, std::vector<int>& obj2_faces
                                ,   std::set<contact> & contacts ) {
    
    for( int f1 : obj1_faces) {             // go through all face-face pairs
        for( int f2 : obj2_faces) {
            if( sat( obj1.m_faces[f1], obj2.m_faces[f2], dir) ) {           //only if the faces actually touch - can also drop this if statement
                process_face_face_contact( obj1, obj2, dir, f1, f2, contacts ); //compare all vertices and edges in the faces
            }
        }
    }
}


//find a face of obj1 that touches obj2
//return it and its neighbors by adding their indices to a list
void get_face_obj_contacts( Polytope &obj1, Polytope &obj2, vec3 &dir, std::vector<int>& obj_faces ) {
    for( int f = 0; f < obj1.m_faces.size(); ++f ) {
        Face &face = obj1.m_faces[f];
        if( sat( face, obj2, dir ) ) {              //find the first face from obj1 that touches obj2
            obj_faces.push_back(f);                    //insert into result list
            auto& neighbors = obj1.get_face_neighbors(f);
            std::copy( std::begin(neighbors), std::end(neighbors), std::back_inserter(obj_faces) );   //also insert its neighbors
            return;
        }
    }
}


//neighboring faces algorithm
void neighboring_faces( Polytope &obj1, Polytope &obj2, vec3 &dir, std::set<contact> & contacts ) {
    std::vector<int> obj1_faces;
    std::vector<int> obj2_faces;

    get_face_obj_contacts(obj1, obj2, dir, obj1_faces );    //get list of faces from obj1 that touch obj2
    get_face_obj_contacts(obj2, obj1, dir, obj2_faces );    //get list of faces from obj2 that touch obj1
    process_face_obj_contacts( obj1, obj2, dir, obj1_faces, obj2_faces, contacts ); //collide them pairwise
}


//compute a list of contact points between two objects
void  contacts( Polytope &obj1, Polytope &obj2, vec3 &dir, std::set<contact> & contacts ) {
    if( dot(dir, dir) < 1.0e-6 ) dir = vec3(0.0f, 1.0f, 0.0f);
    neighboring_faces( obj1, obj2, dir, contacts);
}


};

#endif
