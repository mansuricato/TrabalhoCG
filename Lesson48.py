# -*- coding: utf-8 -*-
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import copy
import math
import numpy

from ArcBall import * 				# ArcBallT and this tutorials set of points/vectors/matrix types
from geometry import *
from graph2 import *
# from matrix import *

PI2 = 2.0*3.1415926535			# 2 * PI (not squared!) 		// PI Squared

# *********************** Globals *********************** 
# Python 2.2 defines these directly
try:
	True
except NameError:
	True = 1==1
	False = 1==0

g_Transform = Matrix4fT ()
g_LastRot = Matrix3fT ()
g_ThisRot = Matrix3fT ()

g_ArcBall = ArcBallT (640, 480)
g_isDragging = False
g_quadratic = None
g_isClicked = True

solidFaces = [];
objectsToDraw = [];

graph = Graph();

angularSpeed = 0.2;



#          blue    green     cyan     red      pink     yellow   white
colors = ([0,0,1], [0,1,0], [0,1,1], [1,0,0], [1,0,1], [1,1,0], [1,1,1])

color_dict = {
	(0,0,1): "blue",
	(0,1,0): "green",
	(0,1,1): "cyan",
	(1,0,0): "red",
	(1,0,1): "pink",
	(1,1,0): "yellow",
	(1,1,1): "white",
	(0.502, 0.502, 0.502): "gray"
}

# A general OpenGL initialization function.  Sets all of the initial parameters. 
def Initialize (Width, Height, argv):				# We call this right after our OpenGL window is created.
	global g_quadratic, solidFaces, objectsToDraw, graph;

	if len(argv) < 2:
		print "Argumento faltando. Por favor, especifique o arquivo na linha de comando."
		return False

	glClearColor(0.0, 0.0, 0.0, 1.0)					# This Will Clear The Background Color To Black
	glClearDepth(1.0)									# Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LEQUAL)								# The Type Of Depth Test To Do
	glEnable(GL_DEPTH_TEST)								# Enables Depth Testing
	glShadeModel (GL_FLAT);								# Select Flat Shading (Nice Definition Of Objects)
	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST) 	# Really Nice Perspective Calculations

	g_quadratic = gluNewQuadric();
	gluQuadricNormals(g_quadratic, GLU_SMOOTH);
	gluQuadricDrawStyle(g_quadratic, GLU_FILL); 
	# Why? this tutorial never maps any textures?! ? 
	# gluQuadricTexture(g_quadratic, GL_TRUE);			# // Create Texture Coords

	# glEnable (GL_LIGHT0)
	# glEnable (GL_LIGHTING)

	# glEnable (GL_COLOR_MATERIAL)

	vertices, faces = getSolidFromFile(argv);
	solidFaces = buildSolidStructure(vertices, faces);
	objectsToDraw.append(solidFaces);

	graph = buildGraph(solidFaces);

	return True

def buildGraph(polygons):
	graph = Graph();

	for polygon in polygons:
		graph.add_vertex(polygon);

	for polygon1 in polygons:
		for polygon2 in polygons:
			if polygon1.points != polygon2.points and doPolygonsHaveACommonEdge(polygon1, polygon2):
				graph.add_edge({polygon1, polygon2});
	return graph;

def doPolygonsHaveACommonEdge(poly1, poly2):
	edges1 = generatePolygonEdges(poly1);
	edges2 = generatePolygonEdges(poly2);

	for line1 in edges1:
		for line2 in edges2:
			if (line1.p1 == line2.p1 and line1.p2 == line2.p2) or (line1.p1 == line2.p2 and line1.p2 == line2.p1):
				return True;
	return False;

def getCommonEdges(poly1, poly2):
	edges1 = generatePolygonEdges(poly1);
	edges2 = generatePolygonEdges(poly2);
	edgesInCommon = [];

	for line1 in edges1:
		for line2 in edges2:
			if (line1.p1 == line2.p1 and line1.p2 == line2.p2) or (line1.p1 == line2.p2 and line1.p2 == line2.p1):
				edgesInCommon.append(line1);
	return edgesInCommon;

def getSolidFromFile(argv):
	global solidFaces, objectsToDraw;
	if len(argv) < 2:
		return None;

	f = open(argv[1]);
	lines = f.readlines();

	line = lines[3].split(" ");
	nVertices = int(line[2]);

	line = lines[7].split(" ");
	nFaces = int(line[2]);

	vertexIndex = 0;
	vertices = [];
	while vertexIndex < nVertices:
		v = lines[10 + vertexIndex].split(" ");
		vertices.append(Point(float(v[0]), float(v[1]), float(v[2])));
		vertexIndex += 1;

	faceIndex = 0;
	faces = [];
	while faceIndex < nFaces:
		face = lines[10 + vertexIndex + faceIndex].split(" ");
		faces.append([int(fac) for fac in face[1:] if str.isdigit(fac)]);
		faceIndex += 1;

	f.close();
	return vertices, faces;

# retorna o array de polígonos que forma o cubo, com base nas variáveis globais acima: vertices e surfaces
def buildSolidStructure(vertices, surfaces):
	i = 0;
	polygons = [];
	for listOfVertexIndexes in surfaces:
		arrayOfPoints = [];
		for vertexIndex in listOfVertexIndexes:
			arrayOfPoints.append(vertices[vertexIndex]);
		polygons.append(Polygon(arrayOfPoints, colors[i % len(colors)]));
		i += 1;

	return polygons;

def Upon_Drag (cursor_x, cursor_y):
	""" Mouse cursor is moving
		Glut calls this function (when mouse button is down)
		and pases the mouse cursor postion in window coords as the mouse moves.
	"""
	global g_isDragging, g_LastRot, g_Transform, g_ThisRot

	if (g_isDragging):
		mouse_pt = Point2fT (cursor_x, cursor_y)
		ThisQuat = g_ArcBall.drag (mouse_pt)						# // Update End Vector And Get Rotation As Quaternion
		g_ThisRot = Matrix3fSetRotationFromQuat4f (ThisQuat)		# // Convert Quaternion Into Matrix3fT
		# Use correct Linear Algebra matrix multiplication C = A * B
		g_ThisRot = Matrix3fMulMatrix3f (g_LastRot, g_ThisRot)		# // Accumulate Last Rotation Into This One
		g_Transform = Matrix4fSetRotationFromMatrix3f (g_Transform, g_ThisRot)	# // Set Our Final Transform's Rotation From This One
	return

def Upon_Click (button, button_state, cursor_x, cursor_y):
	""" Mouse button clicked.
		Glut calls this function when a mouse button is
		clicked or released.
	"""
	global g_isDragging, g_LastRot, g_Transform, g_ThisRot, g_isClicked

	g_isDragging = False
	if (button == GLUT_RIGHT_BUTTON and button_state == GLUT_UP and g_isClicked):
		# Right button click
		g_LastRot = copy.copy (g_ThisRot);

		g_isClicked = False

		mouse_pt = Point2fT (cursor_x, cursor_y)
		g_ArcBall.click (mouse_pt);

		x, y = screenToOGLCoords(cursor_x, cursor_y);

		pickedFace = chooseSurface(x, y, solidFaces);

		if pickedFace != None:
			visited = bfs_keeping_track_of_parents(graph, pickedFace);
			# for v in visited:
			# 	print GetColor(v.color)

			# for nei in graph.vertex_neighbours(pickedFace):
			# 	Visit(nei, pickedFace);

	elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_UP):
		# Left button released
		g_LastRot = copy.copy (g_ThisRot);							# // Set Last Static Rotation To Last Dynamic One
	elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_DOWN):
		# Left button clicked down
		g_LastRot = copy.copy (g_ThisRot);							# // Set Last Static Rotation To Last Dynamic One
		g_isDragging = True											# // Prepare For Dragging
		mouse_pt = Point2fT (cursor_x, cursor_y)
		g_ArcBall.click (mouse_pt);								# // Update Start Vector And Prepare For Dragging
		x, y = screenToOGLCoords(cursor_x, cursor_y);

	return

def dfs_iterative(graph, start):
	visited, stack = set(), [start]
	previous = start;
	while stack:
		vertex = stack.pop();
		if vertex not in visited:
			if not doPolygonsHaveACommonEdge(previous, vertex):
				previous = start;
			visited.add(vertex);
			visit(vertex, previous);
			stack.extend(set(graph.vertex_neighbours(vertex)) - visited);
			previous = vertex;
	return visited

def dfs_recursive(graph, start, parent=None, visited=None):
    if visited is None:
        visited = set();
    visited.add(start);
    if parent == None:
    	parent = start;
    visit(start, parent);
    for next in set(graph.vertex_neighbours(start)) - visited:
        dfs_recursive(graph, next, start, visited);
    return visited;

def bfs(graph, start):
    visited, queue = set(), [start];
    while queue:
        vertex = queue.pop(0);
        if vertex not in visited:
            visited.add(vertex);
            visit(vertex, parent);
            queue.extend(set(graph.vertex_neighbours(vertex)) - visited);
    return visited;

def bfs_keeping_track_of_parents(graph, start):
	parent = {};
	queue = [start];
	visited = set();
	parent[start] = start;
	while queue:
		node = queue.pop(0);
		if node not in visited:
			visited.add(node);

			for adjacent in graph.vertex_neighbours(node): # <<<<< record its parent
				parent[adjacent] = node

			visit(node, parent[node]);
			queue.extend(set(graph.vertex_neighbours(node)) - visited);

def visit(thisPoly, prevPoly):
	if thisPoly != prevPoly:
		n1 = [thisPoly.normal[0], thisPoly.normal[1], thisPoly.normal[2]];
		n2 = [prevPoly.normal[0], prevPoly.normal[1], prevPoly.normal[2]];
		angle = Angle(n1, n2);
		angle = numpy.rad2deg(angle);
		edges1 = generatePolygonEdges(thisPoly);
		edges2 = generatePolygonEdges(prevPoly);
		commonEdges = getCommonEdges(thisPoly, prevPoly);
		if commonEdges == []:
			return
		fixedPoint = commonEdges[0].midpoint();
		axis = numpy.cross(n1, n2);
		thisPoly.transAngle = 1;
		thisPoly.transPoint = fixedPoint;
		thisPoly.transAxis = axis;
		thisPoly.transMaxAngle = angle;
		thisPoly.transParent = prevPoly;
		trans = translateAndRotate(thisPoly.transAngle, thisPoly.transPoint, thisPoly.transAxis, thisPoly.transParent);
		thisPoly.transform = trans;
	return

def resetColors(polygons):
	i = 0;
	for polygon in polygons:
		polygon.color = colors[i % len(colors)];
		i += 1;
	return polygons;

# Recebe o ponto onde o mouse clicou, e um array de polígonos, e retorna o polígono que foi clicado
def chooseSurface(mouseX, mouseY, polygons):
	global objectsToDraw;

	# passar os pontos mouseX e mouseY para a rotação certa
	p1 = [mouseX, mouseY, 2];
	p2 = [mouseX, mouseY, -2];
	rotp1 = Matrix3fMulMatrix3f(g_ThisRot, p1);
	rotp2 = Matrix3fMulMatrix3f(g_ThisRot, p2);
	initialPoint = Point(rotp1[0], rotp1[1], rotp1[2]);
	endPoint = Point(rotp2[0], rotp2[1], rotp2[2]);
	line = Line(initialPoint, endPoint);

	interceptedPolygons = getPolygonsCrossedByLine(line, polygons);
	pickedPolygon = getPolygonClosestToScreen(interceptedPolygons, line);

	if pickedPolygon != None:
		objectsToDraw[0] = resetColors(objectsToDraw[0]);
		objectsToDraw[0][objectsToDraw[0].index(pickedPolygon)].color = [0.502, 0.502, 0.502] # gray
	return pickedPolygon;

# Recebe uma linha e um array de polígonos, e retorna quais polígonos são interceptados pela linha
def getPolygonsCrossedByLine(line, polygons):
	polygonArray = [];
	for polygon in polygons:
		contains, p, t = polygon.doesLineCrossPolygon(line);
		if (contains):
			polygonArray.append(polygon);
	return polygonArray;

# Recebe um array de polígonos, e retorna aquele que estiver mais próximo da tela
def getPolygonClosestToScreen(polygons, clickLine):
	closerPolygon = None;
	distanceToOrigin = 2;

	for polygon in polygons:
		intersection = clickLine.intersectToPlane(polygon);

		if intersection[1] < distanceToOrigin:
			distanceToOrigin = intersection[1];
			closerPolygon = polygon;

	return closerPolygon;


def screenToOGLCoords(cursor_x, cursor_y):
	viewport = glGetDoublev(GL_VIEWPORT);

	cursor_x = float (cursor_x);
	cursor_y = float (viewport[3]) - float (cursor_y);

	cursor_z = glReadPixels(cursor_x, int(cursor_y), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT);

	posX, posY, posZ = gluUnProject(cursor_x, cursor_y, cursor_z, None, None, None);

	return posX, posY;

def translateAndRotate(ang, p, axis, parent):
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixf(parent.transform);
	glTranslate(p[0],p[1],p[2]);
	glRotate(ang, axis[0], axis[1], axis[2]);
	glTranslate(-p[0],-p[1],-p[2]);
	T = glGetDoublev ( GL_MODELVIEW_MATRIX );
	glPopMatrix();
	return T;

def generatePolygonEdges(polygon):
	edges = [];
	for i in range(len(polygon.points) - 1):
		edges.append(Line(polygon.points[i], polygon.points[i+1]));
	edges.append(Line(polygon.points[-1], polygon.points[0]));
	return edges;

def drawSolid(solidFaces):
	global angularSpeed
	i = 0;
	for polygon in solidFaces:

			# ANIMAÇÃO:
		if polygon.transMaxAngle != None:
			polygon.transform = translateAndRotate(polygon.transAngle, polygon.transPoint, polygon.transAxis, polygon.transParent);
			polygon.transAngle += angularSpeed;
			if polygon.transAngle >= polygon.transMaxAngle and angularSpeed > 0:
				angularSpeed *= -1;
			elif polygon.transAngle <= 0 and angularSpeed < 0:
				angularSpeed *= -1;


		glMultMatrixf(polygon.transform);

		glBegin(GL_POLYGON);
		if polygon.color == None:
			glColor3fv(colors[i % len(colors)]);
			i += 1;
		else:
			glColor3fv(polygon.color);
		for point in polygon.points:
			glVertex3f(point[0], point[1], point[2]);
		glEnd();

		glLoadIdentity();
		glTranslatef(0.0,0.0,-6.0);
		glMultMatrixf(g_Transform);
	return

def Draw ():
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				# // Clear Screen And Depth Buffer
	glLoadIdentity();												# // Reset The Current Modelview Matrix
	glTranslatef(0.0,0.0,-6.0);										# // Move  Into The Screen 6.0

	glPushMatrix();													# // NEW: Prepare Dynamic Transform
	glMultMatrixf(g_Transform);										# // NEW: Apply Dynamic Transform
	n = len(objectsToDraw);
	for i in range(n):
		drawSolid(objectsToDraw[i]);

	glPopMatrix();													# // NEW: Unapply Dynamic Transform

	glFlush ();														# // Flush The GL Rendering Pipeline
	glutSwapBuffers()
	return

def GetColor(color):
	return color_dict[tuple(color)];

def Length(v):
	return math.sqrt(Vector3fDot(v, v))

def Angle(v1, v2):
	return math.acos(Vector3fDot(v1, v2) / (Length(v1) * Length(v2)))
