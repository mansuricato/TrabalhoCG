from OpenGL.GL   import *
from OpenGL.GLUT import *
from OpenGL.GLU  import *
import sys,os, time

from libs import *

from ArcBall    import *
from Polihedron import Polihedron
from LerArquivoPly        import PLY
from libs.geometry import Point,Line

from PIL.Image import open

PI2 = 2.0*3.1415926535
window = 0
winX = 640
winY = 480
ESCAPE = '\033'

g_Transform = Matrix4fT ()
g_LastRot = Matrix3fT ()
g_ThisRot = Matrix3fT ()

g_ArcBall = ArcBallT (winX, winY)
g_isDragging = False
g_quadratic = None

g_isFaceSelected = False

POLIEDRY = None
RAY = Line(Point(0.0,0.0,0.0,),Point(1.0,1.0,1.0))

ModelMatrix = None

profundidade = 12.0
translating_rate = 1.0

imageID = 0

def InitGL(Width, Height):				# We call this right after our OpenGL window is created.
    glShadeModel(GL_SMOOTH)				# Enables Smooth Color Shading
    glClearColor(0.0, 0.0, 0.0, 0.5)	# This Will Clear The Background Color To Black
    glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
    glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
    glDepthFunc(GL_LEQUAL)				# The Type Of Depth Test To Do
    glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)  # Really Nice Perspective Calculations

    return True						# // Initialization Went OK


def set_poliedry(poliedry = None):
    global POLIEDRY
    POLIEDRY = poliedry

def Initialize (Width, Height, imageName = "images/water.jpg"):				# We call this right after our OpenGL window is created.
    global g_quadratic, ModelMatrix, imageID

    glClearColor(1.0, 1.0, 1.0, 1.0)			# This Will Clear The Background Color To Black
    glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LEQUAL)				# The Type Of Depth Test To Do
    glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
    glShadeModel (GL_FLAT)				# Select Flat Shading (Nice Definition Of Objects)
    glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST) 	# Really Nice Perspective Calculations

    glEnable (GL_COLOR_MATERIAL)

    imageID = loadImage (imageName)

    return True

def ReSizeGLScene(Width, Height):
    if Height == 0:						# Prevent A Divide By Zero If The Window Is Too Small
	Height = 1

    glViewport(0, 0, Width, Height)		# Reset The Current Viewport And Perspective Transformation
    glMatrixMode(GL_PROJECTION)			# // Select The Projection Matrix
    glLoadIdentity()					# // Reset The Projection Matrix
    gluPerspective(45.0, float(Width)/float(Height), 1, 100.0)

    glMatrixMode (GL_MODELVIEW)	# // Select The Modelview Matrix
    glLoadIdentity ()			# // Reset The Modelview Matrix
    g_ArcBall.setBounds (Width, Height)	# //*NEW* Update mouse bounds for arcball
    return

def keyPressed(*args):
    global g_quadratic
    global profundidade
    global translating_rate
    global POLIEDRY,angularSpeed
    key = args [0]
    if key == ESCAPE:
        gluDeleteQuadric (g_quadratic)
        sys.exit ()
    elif key == 'c':
        POLIEDRY.close()
    elif key == 'w':
        profundidade += translating_rate
    elif key == 's':
        profundidade -= translating_rate

def Upon_Drag (cursor_x, cursor_y):
    global g_isDragging, g_LastRot, g_Transform, g_ThisRot

    if (g_isDragging):
    	mouse_pt = Point2fT (cursor_x, cursor_y)
    	ThisQuat = g_ArcBall.drag (mouse_pt)				        # Update End Vector And Get Rotation As Quaternion
    	g_ThisRot = Matrix3fSetRotationFromQuat4f (ThisQuat)		        # Convert Quaternion Into Matrix3fT
    	g_ThisRot = Matrix3fMulMatrix3f (g_LastRot, g_ThisRot)		        # Accumulate Last Rotation Into This One
    	g_Transform = Matrix4fSetRotationFromMatrix3f (g_Transform, g_ThisRot)	# Set Our Final Transform's Rotation From This One
    return

def get_mouse_position_transform(winX,winY,z1):
    global ModelMatrix
    ModelMatrix = glGetDoublev(GL_MODELVIEW_MATRIX)
    ProjMatrix  = glGetDoublev(GL_PROJECTION_MATRIX)
    Viewport    = glGetIntegerv(GL_VIEWPORT)

    (newX,newY,newZ) = gluUnProject(winX,480 - winY,z1,ModelMatrix,ProjMatrix,Viewport)
    return Point(newX,newY,newZ)


def Upon_Click (button, button_state, cursor_x, cursor_y):
    global g_isDragging, g_LastRot, g_Transform, g_ThisRot
    global POLIEDRY,g_isFaceSelected,RAY, factor, angularSpeed

    if button_state == GLUT_DOWN:
        p_s1 = get_mouse_position_transform(cursor_x,cursor_y,1.0)
        #print(p_s1)
        RAY = Line(Point(0.0,0.0,0.0),p_s1)
        if (g_isFaceSelected == False)and(button == GLUT_LEFT_BUTTON):
            if(POLIEDRY.face_intersect(RAY) != -1):
                POLIEDRY.face_select(POLIEDRY.last_face_clicked)
                g_isFaceSelected = True

    g_isDragging = False

    if (button == GLUT_RIGHT_BUTTON and button_state == GLUT_DOWN and POLIEDRY.isOpened == False):
        if POLIEDRY.face_selected == -1:
            return
        angularSpeed = 0.
        POLIEDRY.animate()
    if (button == GLUT_LEFT_BUTTON and button_state == GLUT_UP):

	g_LastRot = copy.copy (g_ThisRot)					# Set Last Static Rotation To Last Dynamic One
    elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_DOWN):

	g_LastRot = copy.copy (g_ThisRot)					# Set Last Static Rotation To Last Dynamic One
	g_isDragging = True							        # Prepare For Dragging
	mouse_pt = Point2fT (cursor_x, cursor_y)
        g_ArcBall.click (mouse_pt)						# Update Start Vector And Prepare For Dragging
    return

def loadImage(imageName ):

    im = open(imageName)
    try:
            ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGBA", 0, -1)
    except (SystemError, ValueError):
            ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGBX", 0, -1)
    except AttributeError:
            ix, iy, image = im.size[0], im.size[1], im.tostring("raw", "RGBX", 0, -1)

    ID = glGenTextures(1)

    glBindTexture(GL_TEXTURE_2D, ID)
    glPixelStorei(GL_UNPACK_ALIGNMENT,1)

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)
    return ID

def setupTexture():
    global imageID

    glEnable(GL_TEXTURE_2D)

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
    glBindTexture(GL_TEXTURE_2D, imageID)


angularSpeed = 0.
factor = 1./100.

def Draw ():
    global POLIEDRY,RAY,profundidade,g_isFaceSelected
    global angularSpeed,factor

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Clear Screen And Depth Buffer
    glLoadIdentity()				        # Reset The Current Modelview Matrix
    glTranslatef(0.0,0.0,-profundidade)    # Move Left 1.5 Units And Into The Screen 6.0

    ModelMatrix = glGetDoublev(GL_MODELVIEW_MATRIX)

    glMultMatrixf(g_Transform) 		    # NEW: Apply Dynamic Transform

    if POLIEDRY.isAnimated:
        POLIEDRY.open_like_BFS(angularSpeed)
        angularSpeed += factor
        if((angularSpeed >= 1)or(angularSpeed < 0.)):
            POLIEDRY.set_texture()
            if POLIEDRY.face_selected == -1:
                return
            factor *= -1.

    if POLIEDRY.isOpened:
        POLIEDRY.open_like_BFS(1.0)

    if POLIEDRY.useTexture:
        setupTexture()
    else:
        glDisable(GL_TEXTURE_2D)

    POLIEDRY.draw()

    glFlush()                             # Flush The GL Rendering Pipeline
    glutSwapBuffers()
    return

def main():
    global window
    glutInit(sys.argv)

    filename = sys.argv[1]
    imageFileName = sys.argv[2]

    poliedry = Polihedron(PLY(filename))

    set_poliedry(poliedry)


    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)

    glutInitWindowSize(winX,winY)

    glutInitWindowPosition(0, 0)

    window = glutCreateWindow("Trabalho de CG 2017.1")

    glutDisplayFunc(Draw)

    glutIdleFunc(Draw)

    glutReshapeFunc(ReSizeGLScene)

    glutKeyboardFunc(keyPressed)

    glutMouseFunc (Upon_Click)

    glutMotionFunc (Upon_Drag)

    Initialize (winX, winY,imageFileName)

    glutMainLoop()

if __name__ == "__main__":
    print "Hit ESC key to quit."
    main()
