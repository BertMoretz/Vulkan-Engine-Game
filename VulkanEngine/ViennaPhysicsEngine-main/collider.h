#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

using namespace glm;

constexpr float EPS = 1.0e-6f;

struct ICollider {
    ICollider(){};
    virtual vec3 support(vec3 dir) = 0;
};


//Pluecker coordinates for point, line and plane (Eric Lengyel, Foundations of Game Engine Development, Vol 1: Mathematics, 2016)
using pluecker_point = vec4;    // homogeneous 4D coordinates (p | w) where the original point q is 1/w p
using pluecker_plane = vec4;    // 4D plane coordinates [normal | d] where d = -dot(normal,q) for q point on the plane
 
struct pluecker_line {          // 6D line coordinates { direction vector | p0 x p1 }, where p0 and p1 are points on the line
    vec3 dir;
    vec3 moment;
};


//Base struct for all collision shapes
struct Collider : ICollider {
    vec3    m_pos;            //origin in world space
    mat3    m_matRS;          //rotation/scale component of model matrix
    mat3    m_matRS_inverse; 

    Collider( vec3 p = {0,0,0}, mat3 m = mat3(1.f) ) : ICollider() {
        m_pos = p;
        m_matRS = m;
        m_matRS_inverse = inverse( m );
    };
};

//BBox: AABB + Orientation matrix
struct BBox : Collider {
    vec3 m_min, m_max; //Assume these are axis aligned!

    vec3 support(vec3 dir){
        dir = m_matRS_inverse*dir; //find support in model space

        vec3 result;
        result.x = (dir.x>0) ? m_max.x : m_min.x;
        result.y = (dir.y>0) ? m_max.y : m_min.y;
        result.z = (dir.z>0) ? m_max.z : m_min.z;

        return m_matRS*result + m_pos; //convert support to world space
    }
};

//Sphere: NB Does not use RS matrix, scale the radius directly!
struct Sphere : Collider {
    float m_r;

    Sphere( vec3 pos = vec3(0.0f, 0.0f, 0.0f), float radius = 1.0f) : Collider(pos, mat3(1.0f)), m_r(radius) {};

    vec3 support(vec3 dir){
        return normalize(dir)*m_r + m_pos;
    }
};


//a Point3D is a sphere with tiny radius
//can be used with GJK
struct Point3D : Sphere {
    Point3D( vec3 pos = vec3(0.0f, 0.0f, 0.0f) ) : Sphere(pos, EPS) {}
    pluecker_point pluecker() { return { m_pos, 1}; };
};


//a 1d line segment, do not use ith GJK
struct Point : Collider {
    Point( vec3 p ) : Collider(p) {}
    Point & operator=(const Point & l) = default;
    pluecker_point pluecker() { return { m_pos, 1}; };

    vec3 support(vec3 dir) {
        return m_pos; 
    }
};



//Cylinder: Height-aligned with y-axis (rotate using matRS)
struct Cylinder : Collider {
    float m_r, m_y_base, m_y_cap;

    vec3 support(vec3 dir){
        dir = m_matRS_inverse*dir; //find support in model space
        vec3 dir_xz = vec3(dir.x, 0, dir.z);
        vec3 result = normalize(dir_xz)*m_r;
        result.y = (dir.y>0) ? m_y_cap : m_y_base;
        return m_matRS*result + m_pos; //convert support to world space
    }
};


//Capsule: Height-aligned with y-axis
struct Capsule : Collider {
    float m_r, m_y_base, m_y_cap;

    Capsule(    vec3 pos = vec3{0.0f, 0.0f, 0.0f}, mat3 matRS = mat3(1.0f)
            ,   float radius = 0.2f, float yb = -0.5f, float yc = 0.5 ) 
                    : Collider( pos, matRS ), m_r(radius), m_y_base(yb), m_y_cap(yc)  {}

    vec3 support(vec3 dir){
        dir = m_matRS_inverse*dir; //find support in model space
        vec3 result = normalize(dir)*m_r;
        result.y += (dir.y>0) ? m_y_cap : m_y_base;
        return m_matRS*result + m_pos; //convert support to world space
    }
};


//a Line3D is a capsule with tiny radius
//can be used with GJK
struct Line3D : Capsule {
    Line3D( vec3 start = vec3{0.0f, -0.5f, 0.0f}, vec3 end = vec3{0.0f, 0.5f, 0.0f}  ) 
            : Capsule( (start + end)*0.5f, mat3(1.0), 1.0e-6f ) {
        vec3 vector = end - start;
        vec3 dir = normalize( vector );
        if( length(dir) < 0.9f ) return;

        vec3 up = {0.0f, 1.0f, 0.0f};
        vec3 naxis = cross( up, dir );
        if( length( naxis ) < 1.0e-9 )  {
            up = {1.0f, 0.0f, 0.0f};
            naxis = cross( up, dir );
        }
        vec3 xaxis = cross( dir, naxis);
        vec3 zaxis = cross( xaxis, dir);

        m_matRS = mat3( xaxis, dir, zaxis );
        m_matRS = m_matRS * mat3( scale( mat4(1.0f), vec3(1.0f, length( vector ), 1.0f) ) );
        m_matRS_inverse = inverse( m_matRS );
    }

    pluecker_line pluecker() { 
        vec3 dir = m_matRS*vec3(0,1,0);  //capsule in local space along y axis
        return { dir, cross( m_pos, m_pos + dir) }; 
    };

};

//a 1d line segment, do not use ith GJK
struct Line : Collider {
    vec3 m_dir;  //

    Line( vec3 p0, vec3 p1 ) : Collider(p0), m_dir(p1 - p0) {}
    Line & operator=(const Line & l) = default;

    pluecker_line pluecker() const {
        vec3 dir = m_matRS*m_dir;
        return { dir, cross( m_pos, m_pos + dir) }; 
    };
    
    float t( vec3 &point ) const {     //point = pos + t dir
        vec3 d = point - m_pos;
        if( std::abs(m_dir.x)>std::abs(m_dir.y) && std::abs(m_dir.x)>std::abs(m_dir.z)) {
            return d.x / m_dir.x;
        } else if( std::abs(m_dir.y)>std::abs(m_dir.z)) {
            return d.y / m_dir.y;
        }
        return d.z / m_dir.z;
    }

    bool in_segment(vec3 &point) const {
        float distance_point_line = length(cross(pluecker().dir, point) + pluecker().moment) / length(pluecker().dir);
        if( distance_point_line < EPS ) return false;
        float ft = t(point);
        return 0.0f <= ft && ft <= 1.0f;
    }

    vec3 support(vec3 dir) {
        dir = m_matRS_inverse*dir; //find support in model space
        vec3 result = dot( dir, m_dir ) < 0.0f ?  vec3{0,0,0} : m_dir;
        return m_matRS*result + m_pos; //convert support to world space
    }
};



//--------------------------------------------------------------------------------------

struct Polytope; 

struct PolytopePart : ICollider {
    PolytopePart() : ICollider() {};
};


struct VertexData {
    int              m_index;
    std::vector<int> m_faces;       //indices of all faces of this vertex
    std::vector<int> m_neighbors;   //indices of all vertex neighbors of this vertex
    VertexData(int i, std::vector<int> f, std::vector<int> ne) 
        : m_index(i), m_faces(f), m_neighbors(ne) {}
    VertexData & operator=(const VertexData & v) = default;
};


//Vertex of a polytope
//constexpr int MAX_NEIGHBORS = 8;   //adapt the max if you need more neighbors
struct Vertex : PolytopePart {
    Polytope*           m_polytope;
    const VertexData*   m_data;

    Vertex(Polytope* p, const VertexData *d ) : PolytopePart(), m_polytope(p), m_data(d) {}
    Vertex & operator=(const Vertex & v) = default;
    vec3            pointW();
    const std::vector<int> &    neighbors() const { return m_data->m_neighbors; };
    pluecker_point pluecker();
    vec3 support(vec3 dir);
};


struct FaceData {
    int              m_index;
    std::vector<int> m_vertices;          //indices of all vertices of this face
    std::vector<int> m_normal_vertices;   //indices of the points to use to compute the normal of this face
    std::vector<int> m_neighbors;         //indices of the faces that are neighbors of this face

    FaceData(int i, std::vector<int> v, std::vector<int> n, std::vector<int> ne )
         : m_index(i), m_vertices(v), m_normal_vertices(n), m_neighbors(ne) {}
    FaceData & operator=(const FaceData & v) = default;
};


//Face of a polytope
struct Face  : PolytopePart {
    Polytope*        m_polytope;
    const FaceData * m_data;

    Face(Polytope* p, const FaceData *d) : PolytopePart(), m_polytope(p), m_data(d) {}
    Face & operator=( const Face & v) = default;

    bool contains_vertex(int v ) const {
        return std::find( std::begin(m_data->m_vertices), std::end(m_data->m_vertices), v) != std::end(m_data->m_vertices);
    }

    vec3 get_face_normal() const;
    void get_face_points( std::vector<vec3> &points ) const;
    void get_edge_vectors( std::vector<vec3> & vectors ) const ;
    void get_edges( std::vector<Line> & edges );
    const std::vector<int> & face_vertices() const { return m_data->m_vertices; }
    pluecker_plane pluecker();
    bool inside_cell( vec3 &point ) const;
    vec3 support(vec3 dir);
};


struct Polygon;

//Polytope: Just a set of points plus adjencency information
struct Polytope : Collider {
    std::vector<vec3>     m_points;
    std::vector<Vertex>   m_vertices;
    std::vector<Face>     m_faces;
    int                   support_point = -1;

    Polytope( vec3 pos = {0,0,0}, mat3 matRS = mat3(1.0f) ) : Collider(pos, matRS) {}

    //Dumb O(n) support function, just brute force check all points
    vec3 support(vec3 dir) {
        dir = m_matRS_inverse*dir;

        vec3 furthest_point; 
        float max_dot;

        if (m_vertices[0].neighbors().empty()) {           //if( neighbors == nullptr ) {  //replace the if statement so this branch is only taken if no neighbor info available
            furthest_point = m_points[0]; 
            max_dot = dot(furthest_point, dir);
            std::for_each(  std::begin(m_points), std::end(m_points), 
                            [&]( auto &v) {
                                float d = dot(v, dir);
                                if(d>max_dot){
                                    max_dot = d;
                                    furthest_point = v;
                                }
                            } );
        } else {
            //ADD YOUR CODE HERE TO ITERATE THROUGH NEIGHBORS rather than iterate through all points
            if (support_point < 0) {
                support_point = 0;
            }

            furthest_point = m_points[support_point]; 
            max_dot = dot(furthest_point, dir);
            int max = support_point;

            do {
                support_point = max;
                for ( int neighbor : m_vertices[support_point].neighbors() ) {
                    float d = dot(m_points[neighbor], dir);
                    if (d > max_dot) {
                        max_dot = d;
                        furthest_point = m_points[neighbor];
                        max = neighbor;
                    }
                }
            } while( max != support_point );
        }

        vec3 result = m_matRS*furthest_point + m_pos; //convert support to world space
        return result;
    }

    const std::vector<int>& get_face_neighbors( int f ) const;
    const std::vector<int>& get_vertex_neighbors(int v) const;
    void get_face_points( int f, std::vector<vec3> &points ) const;
    vec3 get_face_normal( int f ) const;
    void get_edge_vectors( std::vector<vec3> & edges) const;
    void get_edges( std::vector<Line> & edges );
};


struct Tetrahedron : Polytope {

    const static inline std::vector<VertexData> m_vertices_data =  //4 vertices, each is member of 3 faces and has 3 neighbors
                        {       VertexData{ 0, {0,1,3}, {1,2,3} }  //0
							, 	VertexData{ 1, {0,1,2}, {0,2,3} }  //1
							, 	VertexData{ 2, {0,2,3}, {0,1,3} }  //2
							, 	VertexData{ 3, {1,2,3}, {0,1,2} }  //3
                        };

	const static inline std::vector<FaceData> m_faces_data =  //4 faces, each has 3 vertices (clockwise), 4 vertices for computing normals, 3 neighbor faces
                    {       FaceData{ 0, {0,1,2}, {1,0,2,0}, {1,2,3} }  //0
                        ,   FaceData{ 1, {0,3,1}, {3,0,1,0}, {0,2,3} }  //1
                        ,   FaceData{ 2, {1,3,2}, {3,1,2,1}, {0,1,3} }  //2
                        ,   FaceData{ 3, {2,3,0}, {3,2,0,2}, {0,1,2} }  //3
                    };

    Tetrahedron( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Polytope() {
	    m_points = { p0, p1, p2, p3 };
        for( const auto& data : m_vertices_data ) m_vertices.emplace_back(  this, &data );
        for( const auto& data : m_faces_data ) m_faces.emplace_back( this, &data );
    };
};


//a triangle is a tertrahedron with tiny height
//can be used with GJK
struct Triangle3D : Tetrahedron {
    Triangle3D( vec3 p0, vec3 p1, vec3 p2 )  : Tetrahedron( p0, p1, p2, {0,0,0} ) {
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = EPS * normalize( cross( d0,  d1) );
        m_points[3] = (p0 + p1 + p2)*0.33333f + up;
    };
};


//a box is a polytope with 8 vertices
struct Box : Polytope {
    const static inline std::vector<VertexData> m_vertices_data = { //every vertex is member of 3 faces and has 3 neighbors
                                VertexData{ 0, {0,2,4}, {1,2,4} }   // 0  
							, 	VertexData{ 1, {1,2,4}, {0,3,5} }   // 1
							, 	VertexData{ 2, {0,2,5}, {0,3,6} }   // 2
							, 	VertexData{ 3, {1,2,5}, {1,2,7} }   // 3
                            ,   VertexData{ 4, {0,3,4}, {0,5,6} }   // 4
							, 	VertexData{ 5, {1,3,4}, {1,4,7} }   // 5
							, 	VertexData{ 6, {0,3,5}, {2,4,7} }   // 6
							, 	VertexData{ 7, {1,3,5}, {3,5,6} }   // 7
						};

    const static inline std::vector<FaceData> m_faces_data =  //6 faces, each having 4 vertices (clockwise), 4 vertices for computing normals, 4 neighbor faces
                    {   FaceData{ 0, {0,2,4,6}, {6,2,0,2}, {2,3,4,5} }   //0
                    ,   FaceData{ 1, {1,3,5,7}, {5,1,3,1}, {2,3,4,5} }   //1
                    ,   FaceData{ 2, {0,1,2,3}, {1,0,2,0}, {0,1,4,5} }   //2
                    ,   FaceData{ 3, {4,5,6,7}, {6,4,5,4}, {0,1,4,5} }   //3
                    ,   FaceData{ 4, {0,1,4,5}, {4,0,1,0}, {0,1,2,3} }   //4
                    ,   FaceData{ 5, {2,3,6,7}, {7,3,2,3}, {0,1,2,3} }   //5
                    };

    Box( vec3 pos = vec3(0.0f, 0.0f, 0.0f), mat3 matRS = mat3(1.0f) )  : Polytope( pos, matRS ) {
	    m_points = {    vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f), vec3(-0.5f, -0.5f, 0.5f), vec3(0.5f, -0.5f, 0.5f),
                        vec3(-0.5f,  0.5f, -0.5f), vec3(0.5f,  0.5f, -0.5f), vec3(-0.5f,  0.5f, 0.5f), vec3(0.5f,  0.5f, 0.5f)};

        for( const auto& data : m_vertices_data ) m_vertices.emplace_back(  this, &data );
        for( const auto& data : m_faces_data ) m_faces.emplace_back( this, &data );
    };
};

//a quad is a box with tiny height
//can be used with GJK
struct Quad3D : Box {
    Quad3D( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )  : Box() {   //points should lie in the same plane
        vec3 d0 = p2 - p0;
        vec3 d1 = p1 - p0;
        vec3 up = EPS * normalize( cross( d0,  d1) );
	    m_points = { p0, p1, p2, p3, p0 + up, p1 + up, p2 + up, p3 + up };
    };
};

//a Polygon3D with N vertices, all vertices must lie in the same plane
//the polgon has a 3D body but is infinitely thin
//can be used with GJK
struct Polygon3D : Polytope {
    Polygon3D( std::vector<vec3>& points ) : Polytope() {
        vec3 d0 = points[2] - points[0];
        vec3 d1 = points[1] - points[0];
        vec3 up = EPS * normalize( cross( d0,  d1) );

	    m_points = points;
        for( int i=0; i<points.size(); ++i ) {
            m_points.push_back(points[i] + up);
        }
    }
};


//-------------------------------------------------------------------------------------

//Polytope parts

pluecker_point Vertex::pluecker() { 
    return { m_polytope->m_points[m_data->m_index], 1.0f}; 
};

vec3 Vertex::pointW() {
    return m_polytope->m_points[m_data->m_index];
}

vec3 Vertex::support(vec3 dir) {
    vec3 result = m_polytope->m_points[m_data->m_index];
    return m_polytope->m_matRS * result + m_polytope->m_pos; //convert support to world space
}

//get the normal of the face
vec3 Face::get_face_normal() const {
    auto &n = m_data->m_normal_vertices;
    return cross( m_polytope->m_matRS * (m_polytope->m_points[n[0]] - m_polytope->m_points[n[1]])
                , m_polytope->m_matRS * (m_polytope->m_points[n[2]] - m_polytope->m_points[n[3]] ) );
}

//return a list with the coordinates of the vertices of a given face in world coordinates
void Face::get_face_points( std::vector<vec3> &points ) const {
    for( auto i : m_data->m_vertices) {
        points.push_back( m_polytope->m_matRS * m_polytope->m_points[i] + m_polytope->m_pos );
    }
}

//return a list of vectors going along the edges of the face
void Face::get_edge_vectors( std::vector<vec3> &vectors ) const {
    int v0 = m_data->m_vertices.back();
    for( int v : m_data->m_vertices ) {
        vectors.push_back( m_polytope->m_matRS * (m_polytope->m_points[v] - m_polytope->m_points[v0]) );
        v0 = v;
    }
}

//return a list of edges of this face
void Face::get_edges( std::vector<Line> & edges ) {
    int v0 = m_data->m_vertices.back();
    for( int v : m_data->m_vertices ) {
        edges.emplace_back( m_polytope->m_matRS * m_polytope->m_points[v0], m_polytope->m_matRS * m_polytope->m_points[v] );
        v0 = v;
    }
}

pluecker_plane Face::pluecker() {
    vec3 normal = get_face_normal();
    return { normal, -1.0f * dot( normal, m_polytope->m_points[m_data->m_vertices[0]])};
}

bool Face::inside_cell( vec3 &pointW) const {
    int v0 = face_vertices().back();
    vec3 n = get_face_normal();
    for( int v : face_vertices() ) {
        vec3 p0 = m_polytope->m_vertices[v0].pointW();
        vec3 p1 = m_polytope->m_vertices[v].pointW();
        vec3 p2 = p0 - n;
        vec3 plane_normal = cross( p1 - p0, p2 - p0 ); 
        if( dot( pointW - p0, plane_normal) < 0 ) return false;
        v0 = v;
    }
    return true;
}

vec3 Face::support(vec3 dir) {
    dir = m_polytope->m_matRS_inverse*dir; //find support in model space
    vec3 maxp = m_polytope->m_points[0];
    float max = dot( dir, maxp );
    for( auto i : m_data->m_vertices ) {
        float d = dot( dir, m_polytope->m_points[i] );
        if( d > max ) {
            maxp = m_polytope->m_points[i];
            max = d;
        }
    }
    return m_polytope->m_matRS * maxp + m_polytope->m_pos; //convert support to world space
}


//Polytope functions

//Return all neighboring vertices of a given vertex
const std::vector<int> & Polytope::get_vertex_neighbors(int v) const {
	const Vertex &vertex = m_vertices[v];
    return vertex.m_data->m_neighbors;
}

//return all neighboring faces of a given face
const std::vector<int> & Polytope::get_face_neighbors( int f ) const {
    return m_faces[f].m_data->m_neighbors;
}

//get the normal of a given face
vec3 Polytope::get_face_normal( int f ) const {
    return m_faces[f].get_face_normal();
}

//return a list with the coordinates of the vertices of a given face in world coordinates
void Polytope::get_face_points( int f, std::vector<vec3> &points ) const {
    m_faces[f].get_face_points( points );
}

//return a list of vectors going along the edges of the polytope
void Polytope::get_edge_vectors( std::vector<vec3> & vectors) const {
    for( auto & face : m_faces ) {
        face.get_edge_vectors( vectors );
    }
}

void Polytope::get_edges( std::vector<Line> & edges ) {
    for( auto & face : m_faces ) {
        face.get_edges( edges );
    }
}

