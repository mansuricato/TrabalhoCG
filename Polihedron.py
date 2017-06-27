from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import math

import copy

from libs import matrix
from libs.geometry import Point,Polygon,Line,Box

import numpy as np

class Polihedron(object):

    def __init__(self, PLY = None):

        if PLY == None:
            print "PLY nao fornecido"
            exit(0)

        self.box    = Box()
        self.vertex = PLY.vertex
        self.faces  = PLY.faces
        self.colors = [
                (0.0,0.0,0.5),
                (0.0,0.0,1.0),
                (0.0,0.5,0.0),
                (0.0,0.5,0.5),
                (0.0,0.5,1.0),
                (0.0,1.0,0.0),
                (0.0,1.0,0.5),
                (0.0,1.0,1.0),
                (0.5,0.0,0.0),
                (0.5,0.0,0.5),
                (0.5,0.0,1.0),
                (0.5,0.5,0.0),
                (0.5,0.5,0.5),
                (0.5,0.5,1.0),
                (0.5,1.0,0.0),
                (0.5,1.0,0.5),
                (0.5,1.0,1.0),
                (1.0,0.0,0.0),
                (1.0,0.0,0.5),
                (1.0,0.0,1.0),
                (1.0,0.5,0.0),
                (1.0,0.5,0.5),
                (1.0,0.5,1.0),
                (1.0,1.0,0.0),
                (1.0,1.0,0.5),
                (1.0,1.0,1.0)
                ]

        self.isOpened = False
        self.isAnimated = False
        self.useTexture = False
        self.mapRotate  = matrix.identity()
        self.textureMap = {}
        self.aux = 0
        edges_per_face = []
        points_per_face = []
        points_per_face_orig = []
        polygons = []

        for face in self.faces:
            edges_per_face.append([])
            points_per_face.append([])
            points_per_face_orig.append([])
            for i in xrange(len(face)):
                points_per_face[-1].append(copy.deepcopy(self.vertex[face[i]]))
                points_per_face_orig[-1].append(copy.deepcopy(self.vertex[face[i]]))
                if i  != (len(face)-1):
                    edges_per_face[-1].append((face[i],face[i+1]))
                else:
                    edges_per_face[-1].append((face[i],face[0]))
            polygons.append(Polygon(points_per_face_orig[-1]))

        adjacences_list = []
        edge_between_faces = {}

        for i in xrange(len(edges_per_face)):
            adjacences_list.append([])
            for j in xrange(len(edges_per_face[i])):
                edge = edges_per_face[i][j]
                for k in xrange(len(edges_per_face)):
                    if k != i:
                        if (((edge[1],edge[0]) in edges_per_face[k])or((edge[0],edge[1]) in edges_per_face[k])) :
                            adjacences_list[i].append(k)
                            edge_between_faces[(i,k)] = edge
                            edge_between_faces[(k,i)] = edge
                            break
            if len(edges_per_face[i]) != len(adjacences_list[i]):
                print "Inconsistencia na contrucao do grafico"

        self.edges_per_face = edges_per_face
        self.points_per_face = points_per_face
        self.points_per_face_orig = points_per_face_orig
        self.adjacences_list = adjacences_list
        self.polygons = polygons
        self.edge_between_faces = edge_between_faces
        self.selected = [0 for elem in edges_per_face]
        self.face_selected = -1

        return

    def draw(self):
        points = []

        for i,face in enumerate(self.faces):
            if ( len(face) % 3 == 0):
                glBegin(GL_TRIANGLES)
            elif ( len(face) % 4 == 0):
                glBegin(GL_QUADS)
            else:
                glBegin(GL_POLYGON)

            if not self.useTexture:
                c = self.colors[i % len(self.colors)]
                glColor3f(1.0*c[0],1.0*c[1],1.0*c[2])

            if (self.isOpened or self.isAnimated):
                points = self.points_per_face[i]
            else:
                points = self.points_per_face_orig[i]

            count = 0
            texturePerFace = [0]*len(points)
            for point in points:
                #print(actualPoint)
                if self.useTexture:# and actualPoint not in self.textureMap.keys():
                    p = self.box.normalize(point.transform(self.mapRotate))
                    texturePerFace[count] = p
                    texture = p
                    if count == (len(points)-1) and i not in self.textureMap.keys():
                        self.textureMap[i] = texturePerFace
                    elif i in self.textureMap.keys():
                        textureInThisFace = self.textureMap[i]
                        texture = textureInThisFace[count]
                    #texture = self.box.normalize(wrongSizedTexture)
                    #p = self.box.normalize(point.transform(self.mapRotate))
                    #print(self.aux)
                    #print(round(point.x,2), round(point.y, 2), round(point.z,2))
                    glTexCoord2f(texture.x,texture.y)
                    glVertex3f(point.x,point.y,point.z)
                #elif self.useTexture and actualPoint in self.textureMap.keys():
                    #texture = self.textureMap[actualPoint]
                    #glTexCoord2f(texture.x, texture.y)
                    #glVertex3f(point.x,point.y,point.z)
                else:
                    glVertex3f(point.x,point.y,point.z)
                count += 1
            glEnd()

    def face_intersect(self,ray):
        face_to_select = -1
        smaller_u = 100000000000000000.0

        for i,poly in enumerate(self.polygons):
            result = ray.intersectToPlane(poly)
            if not(result == None) and (poly.contains(result[0])):
                if abs(result[1]) < abs(smaller_u):
                    smaller_u = abs(result[1])
                    face_to_select = i

        self.last_face_clicked = face_to_select
        return face_to_select

    def face_select(self,i):
        self.selected[self.last_face_clicked] = 1.
        self.face_selected = self.last_face_clicked

    def face_unselect(self):
        self.selected[self.face_selected] = 0.
        self.face_selected = -1

    def open_like_BFS(self,alpha):
        Q = []
        visite1 = [False for i in self.faces]
        transf_vec = [None for i in self.faces]
        altura = [0 for i in self.faces]

        q0 = self.face_selected
        Q.append(q0)
        visite1[q0] = True
        transf_vec[q0] = matrix.identity()
        altura[q0] = 1.

        self.points_per_face[q0] = copy.deepcopy(self.points_per_face_orig[q0])

        while len(Q) > 0:
            q1 = Q.pop(0)
            for v in self.adjacences_list[q1]:
                if visite1[v] == False :
                    visite1[v] = True
                    Q.append(v)
                    altura[v] = altura[q0] + 1

                    N1 = self.polygons[q1].normal
                    N2 = self.polygons[v].normal

                    (np1,np2) = self.edge_between_faces[(q1,v)]

                    v0 = self.vertex[np1]
                    v1 = self.vertex[np2]

                    edge = Line(v0,v1)
                    ang = np.rad2deg(math.acos(N1.dotProd(N2)))

                    axis = edge.dir

                    if axis.tripleProd(N1,N2) > 0:
                        ang = -ang

                    ang = alpha*ang

                    R = matrix.translateAndRotate(ang,v1,edge.dir)

                    transf_vec[v] = matrix.dot(transf_vec[q1],R)

                    for i in xrange(len(self.points_per_face[v])):
                        self.points_per_face[v][i] =\
                        self.points_per_face_orig[v][i].transform(transf_vec[v])

        if self.useTexture:
            axis_z = Point(0.,0.,1.)

            angle_z = axis_z.dotProd(self.polygons[q0].normal)
            axis_r = axis_z.crossProd(self.polygons[q0].normal)

            self.mapRotate = matrix.translateAndRotate(angle_z,axis_r,self.points_per_face[q0][0])

            for points in self.points_per_face:
                for point in points:
                    self.box.add(point.transform(self.mapRotate))

            self.box.setParameters()

    def open(self):
        self.isOpened = True

    def close(self):
        self.isOpened = False

    def set_texture(self):
        self.useTexture = True

    def unset_texture(self):
        self.useTexture = False

    def animate(self):
        self.isAnimated = True

    def static(self):
        self.isAnimated = False
