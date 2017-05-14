from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from Geometry import *
import sys
import copy
from math import cos, sin

from ArcBall import * 				# ArcBallT and this tutorials set of points/vectors/matrix types

PI2 = 2.0*3.1415926535
faces_triangulares = {}

#assign the vertices of the octahedron
vv=( (0.0,1.0,-1.0),   #v01
     (1.0,1.0,0.0),    #v02
     (-1.0,1.0,0.0),   #v03
     (0.0,1.0,1.0),    #v04
     (0.0,-1.0,1.0),   #v05
     (-1.0,0.0,1.0),   #v06
     (0.0,-1.0,-1.0),  #v07
     (1.0,0.0,-1.0),   #v08
     (1.0,0.0,1.0),    #v09
     (-1.0,0.0,-1.0),  #v10
     (1.0,-1.0,0.0),   #v11
     (-1.0,-1.0,0.0) ) #v12

g_Transform = Matrix4fT ()
g_LastRot = Matrix3fT ()
g_ThisRot = Matrix3fT ()

g_ArcBall = ArcBallT (640, 480)
g_isDragging = False
g_quadratic = None

ESCAPE = '\033'
window = 0

def Initialize (Width, Height):				# We call this right after our OpenGL window is created.
    global g_quadratic

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
    gluQuadricTexture(g_quadratic, GL_TRUE);			# // Create Texture Coords

    return True


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
    global g_isDragging, g_LastRot, g_Transform, g_ThisRot

    g_isDragging = False
    if (button == GLUT_RIGHT_BUTTON and button_state == GLUT_UP):
        # Right button click
        g_LastRot = Matrix3fSetIdentity ();							# // Reset Rotation
        g_ThisRot = Matrix3fSetIdentity ();							# // Reset Rotation
        g_Transform = Matrix4fSetRotationFromMatrix3f (g_Transform, g_ThisRot);	# // Reset Rotation
    elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_UP):
        # Left button released
        g_LastRot = copy.copy (g_ThisRot);							# // Set Last Static Rotation To Last Dynamic One
    elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_DOWN):
        # Left button clicked down
        g_LastRot = copy.copy (g_ThisRot);							# // Set Last Static Rotation To Last Dynamic One
        g_isDragging = True											# // Prepare For Dragging
        mouse_pt = Point2fT (cursor_x, cursor_y)
        g_ArcBall.click (mouse_pt);								# // Update Start Vector And Prepare For Dragging

    return

def desenhaIcosaedro():
    global faces_triangulares
    #f1
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 1, 1);
    glVertex3fv(vv[0])
    #glColor3f(1, 1, 1);
    glVertex3fv(vv[1])
    #glColor3f(1, 1, 1);
    glVertex3fv(vv[2])
    faces_triangulares[1] = Triangle(Point(0.0,1.0,-1.0), Point(1.0,1.0,0.0), Point(-1.0,1.0,0.0))
    glEnd()

    #f2
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 0, 0);
    glVertex3fv(vv[3])
    #glColor3f(2, 1, 1);
    glVertex3fv(vv[2])
    #glColor3f(3, 3, 2);
    glVertex3fv(vv[1])
    faces_triangulares[2] = Triangle(Point(0.0,1.0,1.0),Point(-1.0,1.0,0.0), Point(1.0,1.0,0.0))
    glEnd()

    #f3
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 0.25, 0);
    glVertex3fv(vv[3])
    #glColor3f(0, 0, 1);
    glVertex3fv(vv[4])
    #glColor3f(0, 1, 1);
    glVertex3fv(vv[5])
    faces_triangulares[3] = Triangle(Point(0.0,1.0,1.0),Point(0.0,-1.0,1.0), Point(-1.0,0.0,1.0))
    glEnd()

    #f4
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 0.5, 0);
    glVertex3fv(vv[3])
    #glColor3f(0, 0, 3);
    glVertex3fv(vv[8])
    #glColor3f(1, 3, 1);
    glVertex3fv(vv[4])
    faces_triangulares[4] = Triangle(Point(0.0,1.0,1.0),Point(1.0,0.0,1.0), Point(-1.0,0.0,1.0))
    glEnd()

    #f5
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 0.75,0);
    glVertex3fv(vv[0])
    #glColor3f(1, 0, 1);
    glVertex3fv(vv[6])
    #glColor3f(1, 2, 1);
    glVertex3fv(vv[7])
    faces_triangulares[5] = Triangle(Point(0.0,1.0,-1.0),Point(0.0,-1.0,-1.0), Point(1.0,0.0,-1.0))
    glEnd()

    #f6
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 1, 0);
    glVertex3fv(vv[0])
    #glColor3f(3, 0, 3);
    glVertex3fv(vv[9])
    #glColor3f(3, 0, 2);
    glVertex3fv(vv[6])
    faces_triangulares[6] = Triangle(Point(0.0,1.0,-1.0),Point(-1.0,0.0,-1.0), Point(0.0,-1.0,-1.0))
    glEnd()

    #f7
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 1, 0.25);
    glVertex3fv(vv[4])
    #glColor3f(2, 0, 1);
    glVertex3fv(vv[10])
    #glColor3f(1, 1, 1);
    glVertex3fv(vv[11])
    faces_triangulares[7] = Triangle(Point(0.0,-1.0,1.0), Point(1.0,-1.0,0.0), Point(-1.0,-1.0,0.0))
    glEnd()

    #f8
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 1, 0.75);
    glVertex3fv(vv[6])
    #glColor3f(1, 3, 1);
    glVertex3fv(vv[11])
    #glColor3f(2, 2, 1);
    glVertex3fv(vv[10])
    faces_triangulares[8] = Triangle(Point(0.0,-1.0,-1.0), Point(-1.0, -1.0, 0.0),Point(1.0, -1.0, 0.0))
    glEnd()

    #f9
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 0.3, 0.3);
    glVertex3fv(vv[2])
    #glColor3f(2, 0, 1);
    glVertex3fv(vv[5])
    #glColor3f(0, 2, 1);
    glVertex3fv(vv[9])
    faces_triangulares[9] = Triangle(Point(-1.0,1.0,0.0), Point(-1.0,0.0,1.0), Point(-1.0,0.0,-1.0))
    glEnd()

    #f10
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(1, 0.3, 0.75);
    glVertex3fv(vv[11])
    #glColor3f(1, 2, 2);
    glVertex3fv(vv[9])
    #glColor3f(4, 2, 1);
    glVertex3fv(vv[5])
    faces_triangulares[10] = Triangle(Point(-1.0,-1.0,0.0), Point(-1.0,0.0,-1.0), Point(-1.0,0.0,1.0))
    glEnd()

    #f11
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.75, 0.5);
    glVertex3fv(vv[1])
    #glColor3f(3, 0, 0);
    glVertex3fv(vv[7])
    #glColor3f(1, 3, 2);
    glVertex3fv(vv[8])
    faces_triangulares[11] = Triangle(Point(1.0,1.0,0.0), Point(1.0,0.0,-1.0), Point(1.0,0.0,1.0))
    glEnd()

    #f12
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.25, 0.25);
    glVertex3fv(vv[10])
    #glColor3f(1, 0, 0);
    glVertex3fv(vv[8])
    #glColor3f(0, 0, 1);
    glVertex3fv(vv[7])
    faces_triangulares[12] = Triangle(Point(1.0,-1.0,0.0), Point(1.0,0.0,1.0), Point(1.0,0.0,-1.0))
    glEnd()

    #f13
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.5, 0.5);
    glVertex3fv(vv[3])
    #glColor3f(2, 0, 1);
    glVertex3fv(vv[5])
    #glColor3f(0, 0, 1);
    glVertex3fv(vv[2])
    faces_triangulares[13] = Triangle(Point(0.0,1.0,1.0), Point(-1.0,0.0,1.0), Point(-1.0,1.0,0.0))
    glEnd()

    #f14
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.75, 0.75);
    glVertex3fv(vv[3])
    #glColor3f(2, 0, 0);
    glVertex3fv(vv[1])
   # glColor3f(0, 0, 2);
    glVertex3fv(vv[8])
    faces_triangulares[14] = Triangle(Point(0.0,1.0,1.0), Point(1.0,1.0,0.0), Point(1.0,0.0,1.0))
    glEnd()

    #f15
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.2, 0.8);
    glVertex3fv(vv[0])
    #glColor3f(0, 2, 0);
    glVertex3fv(vv[2])
    #glColor3f(0, 1, 1);
    glVertex3fv(vv[9])
    faces_triangulares[15] = Triangle(Point(0.0,1.0,-1.0), Point(-1.0,1.0,0.0), Point(-1.0,0.0,-1.0))
    glEnd()

    #f16
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 1, 0.5);
    glVertex3fv(vv[0])
    #glColor3f(1, 0, 0);
    glVertex3fv(vv[7])
    #glColor3f(2, 2, 2);
    glVertex3fv(vv[1])
    faces_triangulares[16] = Triangle(Point(0.0, 1.0, -1.0), Point(1.0,0.0,-1.0), Point(1.0,1.0,0.0))
    glEnd()

    #f17
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.25, 0.75);
    glVertex3fv(vv[6])
    #glColor3f(2, 0, 1);
    glVertex3fv(vv[9])
    #glColor3f(0, 2, 1);
    glVertex3fv(vv[11])
    faces_triangulares[17] = Triangle(Point(0.0,-1.0,-1.0), Point(-1.0,0.0,-1.0), Point(-1.0,-1.0,0.0))
    glEnd()

    #f18
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.75, 1);
    glVertex3fv(vv[6])
    #glColor3f(2, 0, 0);
    glVertex3fv(vv[10])
    #glColor3f(1, 1, 2);
    glVertex3fv(vv[7])
    faces_triangulares[18] = Triangle(Point(0.0,-1.0,-1.0), Point(1.0,-1.0,0.0), Point(1.0,0.0,-1.0))
    glEnd()

    #f19
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.2, 0.6);
    glVertex3fv(vv[4])
    glVertex3fv(vv[11])
    glVertex3fv(vv[5])
    faces_triangulares[19] = Triangle(Point(0.0,-1.0,1.0), Point(-1.0,-1.0,0.0), Point(-1.0,0.0,1.0))
    glEnd()

    #f20
    glBegin(GL_TRIANGLE_FAN)
    glColor3f(0, 0.6, 0.75);
    glVertex3fv(vv[4])
    glVertex3fv(vv[8])
    glVertex3fv(vv[10])
    faces_triangulares[20] = Triangle(Point(0.0,-1.0,1.0), Point(1.0,0.0,1.0), Point(1.0,-1.0,0.0))
    glEnd()


def Draw ():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				# // Clear Screen And Depth Buffer
    glLoadIdentity();												# // Reset The Current Modelview Matrix
    glTranslatef(0.0,0.0,-6.0);									# // Move Left 1.5 Units And Into The Screen 6.0

    glPushMatrix();													# // NEW: Prepare Dynamic Transform
    glMultMatrixf(g_Transform);										# // NEW: Apply Dynamic Transform
    desenhaIcosaedro();
    glPopMatrix();													# // NEW: Unapply Dynamic Transform

    glLoadIdentity();												# // Reset The Current Modelview Matrix

    glFlush ();														# // Flush The GL Rendering Pipeline
    glutSwapBuffers()
    return

def InitGL(Width, Height):  # We call this right after our OpenGL window is created.

    glShadeModel(GL_SMOOTH)  # Enables Smooth Color Shading
    glClearColor(0.0, 0.0, 0.0, 0.5)  # This Will Clear The Background Color To Black
    glClearDepth(1.0)  # Enables Clearing Of The Depth Buffer
    glEnable(GL_DEPTH_TEST)  # Enables Depth Testing
    glDepthFunc(GL_LEQUAL)  # The Type Of Depth Test To Do
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)  # Really Nice Perspective Calculations

    return True  # // Initialization Went OK

def ReSizeGLScene(Width, Height):
    if Height == 0:  # Prevent A Divide By Zero If The Window Is Too Small
        Height = 1

    glViewport(0, 0, Width, Height)  # Reset The Current Viewport And Perspective Transformation
    glMatrixMode(GL_PROJECTION)  # // Select The Projection Matrix
    glLoadIdentity()  # // Reset The Projection Matrix
    gluPerspective(45.0, float(Width) / float(Height), 1, 100.0)

    glMatrixMode(GL_MODELVIEW);  # // Select The Modelview Matrix
    glLoadIdentity();  # // Reset The Modelview Matrix
    g_ArcBall.setBounds(Width, Height)  # //*NEW* Update mouse bounds for arcball
    return

def keyPressed(*args):
    global g_quadratic
    # If escape is pressed, kill everything.
    key = args[0]
    if key == ESCAPE:
        gluDeleteQuadric(g_quadratic)
        sys.exit()

def main():
    global window
    glutInit(sys.argv)

    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)

    glutInitWindowSize(640, 480)

    glutInitWindowPosition(0, 0)

    window = glutCreateWindow("Icosaedro")

    glutDisplayFunc(Draw)

    glutIdleFunc(Draw)

    glutReshapeFunc(ReSizeGLScene)

    glutKeyboardFunc(keyPressed)

    glutMouseFunc(Upon_Click)

    glutMotionFunc(Upon_Drag)

    Initialize(640, 480)
    glutMainLoop()

if __name__ == "__main__":
    print "Hit ESC key to quit."
    main()
