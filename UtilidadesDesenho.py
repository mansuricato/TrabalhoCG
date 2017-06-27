from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from numpy import *
import sys
import time
import copy
import math
import numpy

import random as rand
from Geometry import *
from graph2 import *

from ArcBall import * 				# ArcBallT and this tutorials set of points/vectors/matrix types

try:
    from PIL import Image
except ImportError as err:
    from Image import open

PI2 = 2.0*3.1415926535

g_Transform = Matrix4fT ()
g_LastRot = Matrix3fT ()
g_ThisRot = Matrix3fT ()

g_ArcBall = ArcBallT (640, 480)
g_isDragging = False
g_isOpening = False
g_isNotMoving = True
g_quadratic = None
g_notRotated = True
imageID = 1

facesDoPoliedro = []
objetosADesenhar = []

grafo = Graph()

velocidadeAngular = 0.9

cima = -999.9
direita = -999.9
baixo = 999.9
esquerda = 999.9

cores = ([rand.random(),rand.random(),rand.random()], [rand.random(),rand.random(),rand.random()], [rand.random(),rand.random(),rand.random()], [rand.random(),rand.random(),rand.random()], [rand.random(),rand.random(),rand.random()], [rand.random(),rand.random(),rand.random()], [rand.random(),rand.random(),rand.random()])

def Initialize (Width, Height, entrada):				# We call this right after our OpenGL window is created.
	global g_quadratic, facesDoPoliedro, objetosADesenhar, grafo, imageID

	if len(entrada) < 2:
	    print('Nenhum arquivo foi passado, por favor, entre com um arquivo.')
	    return False

	glClearColor(0.0, 0.0, 0.0, 1.0)					# This Will Clear The Background Color To Black
	glClearDepth(1.0)									# Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LEQUAL)								# The Type Of Depth Test To Do
	glEnable(GL_DEPTH_TEST)								# Enables Depth Testing
	glShadeModel (GL_FLAT)								# Select Flat Shading (Nice Definition Of Objects)

	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST) 	# Really Nice Perspective Calculations

	glEnable(GL_TEXTURE_2D)
	loadImage()

	g_quadratic = gluNewQuadric()
	gluQuadricNormals(g_quadratic, GLU_SMOOTH)
	gluQuadricDrawStyle(g_quadratic, GLU_FILL)

	vertices, faces = RecebePoliedroDoArquivo(entrada)
	facesDoPoliedro = ConstroiPoliedro(vertices, faces)
	objetosADesenhar.append(facesDoPoliedro)

	grafo = ConstroiGrafo(facesDoPoliedro)

	#gluQuadricTexture(g_quadratic, GL_TRUE)			# // Create Texture Coords

	return True

def loadImage():
	image = Image.open("nekomimi.jpg")
	
	ix = image.size[0]
	iy = image.size[1]
	image = image.tostring("raw", "RGBX", 0, -1)

	glPixelStorei(GL_UNPACK_ALIGNMENT,1)
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)

	glGenerateMipmap(GL_TEXTURE_2D, GL_RGB, ix, iy, GL_UNSIGNED_BYTE, image)
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)

def ConstroiGrafo(poligonos):
	grafoEmConstrucao = Graph()

	for poligono in poligonos:
		grafoEmConstrucao.add_vertex(poligono)

	for primeiroPoligono in poligonos:
		for segundoPoligono in poligonos:
			if primeiroPoligono.points != segundoPoligono.points and VerificaSePoligonosPossuemArestasEmComum(primeiroPoligono, segundoPoligono):
				grafoEmConstrucao.add_edge({primeiroPoligono, segundoPoligono})
	return grafoEmConstrucao

def VerificaSePoligonosPossuemArestasEmComum(primeiroPoligono, segundoPoligono):
	primeiraAresta = GeraArestasDoPoligono(primeiroPoligono)
	segundaAresta = GeraArestasDoPoligono(segundoPoligono)

	for linhaPrimeiraAresta in primeiraAresta:
		for linhaSegundaAresta in segundaAresta:
			if (linhaPrimeiraAresta.p1 == linhaSegundaAresta.p1 and linhaPrimeiraAresta.p2 == linhaSegundaAresta.p2) or (linhaPrimeiraAresta.p1 == linhaSegundaAresta.p2 and linhaPrimeiraAresta.p2 == linhaSegundaAresta.p1):
				return True
	return False

def RetornaArestasEmComum(primeiroPoligono, segundoPoligono):
	primeiraAresta = GeraArestasDoPoligono(primeiroPoligono)
	segundaAresta = GeraArestasDoPoligono(segundoPoligono)
	arestasEmComum = []

	for linhaPrimeiraAresta in primeiraAresta:
		for linhaSegundaAresta in segundaAresta:
			if (linhaPrimeiraAresta.p1 == linhaSegundaAresta.p1 and linhaPrimeiraAresta.p2 == linhaSegundaAresta.p2) or (linhaPrimeiraAresta.p1 == linhaSegundaAresta.p2 and linhaPrimeiraAresta.p2 == linhaSegundaAresta.p1):
				arestasEmComum.append(linhaPrimeiraAresta)
	return arestasEmComum

def RecebePoliedroDoArquivo(argv):
	global facesDoPoliedro, objetosADesenhar
	if len(argv) < 2:
		return None

	arquivoEntrada = open(argv[1])
	linhasDoArquivo = arquivoEntrada.readlines()

	linha = linhasDoArquivo[3].split(" ")
	numeroVertices = int(linha[2])

	linha = linhasDoArquivo[7].split(" ")
	numeroFaces = int(linha[2])

	indiceVertice = 0
	vertices = []
	while indiceVertice < numeroVertices:
		vertice = linhasDoArquivo[10 + indiceVertice].split(" ")
		vertices.append(Point(float(vertice[0]), float(vertice[1]), float(vertice[2])))
		indiceVertice += 1

	indiceFaces = 0
	faces = []
	while indiceFaces < numeroFaces:
		face = linhasDoArquivo[10 + indiceVertice + indiceFaces].split(" ")
		faces.append([int(fac) for fac in face[1:] if str.isdigit(fac)])
		indiceFaces += 1

	arquivoEntrada.close()
	return vertices, faces

def ConstroiPoliedro(vertices, superficies):
	count = 0
	poligonos = []
	for indiceDeVertices in superficies:
		pontos = []
		for indiceVertice in indiceDeVertices:
			pontos.append(vertices[indiceVertice])
		poligonos.append(Polygon(pontos, cores[count % len(cores)]))
		count += 1

	return poligonos

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
	global g_isDragging, g_isOpening, g_isNotMoving, g_LastRot, g_Transform, g_ThisRot, velocidadeAngular, g_notRotated

	g_isDragging = False
	if (button == GLUT_RIGHT_BUTTON and button_state == GLUT_UP and not g_notRotated):
	    # Right button click
	    #g_LastRot = Matrix3fSetIdentity ()							# // Reset Rotation
	    #g_ThisRot = Matrix3fSetIdentity ()							# // Reset Rotation
	    #g_Transform = Matrix4fSetRotationFromMatrix3f (g_Transform, g_ThisRot)	# // Reset Rotation
            if (g_isOpening and g_isNotMoving):
                velocidadeAngular = 0.0
                g_isNotMoving = False
            elif (g_isOpening and not g_isNotMoving):
                velocidadeAngular = 0.1
                g_isNotMoving = True
            else:
                g_isOpening = True
                g_isNotMoving = False
                oglx, ogly = ScreenToOGLCoords(cursor_x, cursor_y)
                faceInterceptada = RetornaFaceInterceptadaPelaReta(oglx, ogly, facesDoPoliedro)

                if faceInterceptada != None:
                    visited = bfs_keeping_track_of_parents(grafo, faceInterceptada)
	elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_UP):
	    # Left button released
            g_notRotated = False
	    g_LastRot = copy.copy (g_ThisRot)							# // Set Last Static Rotation To Last Dynamic One
	elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_DOWN):
	    # Left button clicked down
            g_notRotated = False
	    g_LastRot = copy.copy (g_ThisRot)							# // Set Last Static Rotation To Last Dynamic One
	    g_isDragging = True											# // Prepare For Dragging
	    mouse_pt = Point2fT (cursor_x, cursor_y)
	    g_ArcBall.click (mouse_pt)								# // Update Start Vector And Prepare For Dragging

	return

def bfs(graph, start):
	visited, queue = set(), [start]
	while queue:
	    vertex = queue.pop(0)
	    if vertex not in visited:
	        visited.add(vertex)
	        Visit(vertex, parent)
	        queue.extend(set(graph.vertex_neighbours(vertex)) - visited)
	return visited

def bfs_keeping_track_of_parents(graph, start):
	parent = {}
	queue = [start]
	visited = set()
	parent[start] = start
	while queue:
		node = queue.pop(0)
		if node not in visited:
			visited.add(node)

			for adjacent in graph.vertex_neighbours(node): # <<<<< record its parent
				parent[adjacent] = node

			Visit(node, parent[node])
			queue.extend(set(graph.vertex_neighbours(node)) - visited)

def Visit(poligonoAtual, poligonoAnterior):
	if poligonoAtual != poligonoAnterior:
		normalPoligonoAtual = [poligonoAtual.normal[0], poligonoAtual.normal[1], poligonoAtual.normal[2]]
		normalPoligonoAnterior = [poligonoAnterior.normal[0], poligonoAnterior.normal[1], poligonoAnterior.normal[2]]
		angulo = Angle(normalPoligonoAtual, normalPoligonoAnterior)
		angulo = numpy.rad2deg(angulo)
		primeiraAresta = GeraArestasDoPoligono(poligonoAtual)
		segundaAresta = GeraArestasDoPoligono(poligonoAnterior)
		arestasEmComum = RetornaArestasEmComum(poligonoAtual, poligonoAnterior)
		if arestasEmComum == []:
			return
		pontoFixo = arestasEmComum[0].midpoint()
		eixo = numpy.cross(normalPoligonoAtual, normalPoligonoAnterior)
		poligonoAtual.transAngle = 1
		poligonoAtual.transPoint = pontoFixo
		poligonoAtual.transAxis = eixo
		poligonoAtual.transMaxAngle = angulo
		poligonoAtual.transParent = poligonoAnterior
		trans = translateAndRotate(poligonoAtual.transAngle, poligonoAtual.transPoint, poligonoAtual.transAxis, poligonoAtual.transParent)
		poligonoAtual.transform = trans
	return

def percorreEncontrandoMaximo():
	global grafo, esquerda, direita, cima, baixo

	for poligono in grafo.vertices():
#		print(poligono.transform)
		for vertice in poligono.points:
			transformNp = numpy.array(poligono.transform)
			v = numpy.dot(vertice.np(),transformNp)
			print(v)
			if v[0] < esquerda:
				esquerda = v[0]
			if v[0] > direita:
				direita = v[0]
			if v[1] < baixo:
				baixo = v[1]
			if v[1] > cima:
				cima = v[1]
		print('lalalalala')

	print('esquerda ' + str(esquerda))
	print('direita ' + str(direita))
	print('cima ' + str(cima))
	print('baixo ' + str(baixo))

def translateAndRotate(ang, p, eixo, parent):
	glPushMatrix()
	glLoadIdentity()
	glMultMatrixf(parent.transform)
	glTranslate(p[0],p[1],p[2])
	glRotate(ang, eixo[0], eixo[1], eixo[2])
	glTranslate(-p[0],-p[1],-p[2])
	T = glGetDoublev ( GL_MODELVIEW_MATRIX )
	glPopMatrix()
	return T

def RetornaFaceInterceptadaPelaReta(mouseX, mouseY, poligonos):
	global objetosADesenhar

	p2 = [mouseX, mouseY, -2]
	p2Certo = Matrix3fMulMatrix3f(g_ThisRot, p2)
	pontoDestino = Point(p2Certo[0], p2Certo[1], p2Certo[2])
	linha = Line(Point(0,0,0), pontoDestino)

	poligonoMaisProximo = RetornaPoligonoMaisProximoInterceptadaPelaReta(poligonos, linha)

	return poligonoMaisProximo

def RetornaPoligonoMaisProximoInterceptadaPelaReta(poligonos, retaDoClique):
	poligonoMaisProximo = 0
	menorDistancia = 999.9
	poligonoAtual = 0
	for poligono in poligonos:
		interseccao = poligono.doesLineCrossPolygon(retaDoClique)
		if (interseccao[0]):
			if (interseccao[2] < menorDistancia):
				poligonoMaisProximo = poligono
		poligonoAtual += 1
	return poligonoMaisProximo

def ScreenToOGLCoords(cursor_x, cursor_y):
	viewport = glGetDoublev(GL_VIEWPORT)

	cursor_x = float (cursor_x)
	cursor_y = float (viewport[3]) - float (cursor_y)

	cursor_z = glReadPixels(cursor_x, int(cursor_y), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT)

	posX, posY, posZ = gluUnProject(cursor_x, cursor_y, cursor_z, None, None, None)

	return posX, posY

def GeraArestasDoPoligono(poligono):
	arestas = []
	for i in range(len(poligono.points) - 1):
		arestas.append(Line(poligono.points[i], poligono.points[i+1]))
	arestas.append(Line(poligono.points[-1], poligono.points[0]))
	return arestas

def desenhaPoliedro(facesDoPoliedro):
	global velocidadeAngular, esquerda, direita, cima, baixo
	for poligono in facesDoPoliedro:

		if poligono.transMaxAngle != None:
			poligono.transform = translateAndRotate(poligono.transAngle, poligono.transPoint, poligono.transAxis, poligono.transParent)
			poligono.transAngle += velocidadeAngular
			if poligono.transAngle >= poligono.transMaxAngle and velocidadeAngular > 0:
				velocidadeAngular = 0
				percorreEncontrandoMaximo() #cima,baixo,direita,esquerda sao os maximos
				print('cheguei la')
			elif poligono.transAngle <= 0 and velocidadeAngular < 0:
				velocidadeAngular *= -1

		deltaX = direita - esquerda
		deltaY = cima - baixo

		glMultMatrixf(poligono.transform)

		glBegin(GL_POLYGON)

		pointAntigo = Point(1, 1)

		if poligono.color == None:
			glColor3fv(rand.random())
		else:
			glColor3fv(poligono.color)
		for point in poligono.points:
			transformNp = numpy.array(poligono.transform)
			v = numpy.dot(point.np(),transformNp)

			if poligono.transAngle >= poligono.transMaxAngle and velocidadeAngular == 0:
#
				print(v)
				glTexCoord2f((v[0])/deltaX, (v[1])/deltaY)
			glVertex3f(point[0], point[1], point[2])

		glEnd()

		glLoadIdentity()
		glTranslatef(0.0,0.0,-6.0)
		glMultMatrixf(g_Transform)

	return

def Draw ():
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)				# // Clear Screen And Depth Buffer
	glLoadIdentity()												# // Reset The Current Modelview Matrix
	glTranslatef(0.0,0.0,-6.0)									# // Move Into The Screen 6.0

	glPushMatrix()													# // NEW: Prepare Dynamic Transform
	glMultMatrixf(g_Transform)										# // NEW: Apply Dynamic Transform
	n = len(objetosADesenhar)

	for i in range(n):
		desenhaPoliedro(objetosADesenhar[i])

	glPopMatrix()													# // NEW: Unapply Dynamic Transform

	glFlush ()														# // Flush The GL Rendering Pipeline
	glutSwapBuffers()
	return

def Length(vertice):
	return math.sqrt(Vector3fDot(vertice, vertice))

def Angle(v1, v2):
	return math.acos(Vector3fDot(v1, v2) / (Length(v1) * Length(v2)))

if __name__ == "__main__":
    print "Hit ESC key to quit."
    main()
