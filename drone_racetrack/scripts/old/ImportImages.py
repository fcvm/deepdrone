
from PIL import Image, ImageTk






def LoadDynamicImgs():

    # RACETRACK 2D VIEW

    IconSize = (300, 300)

    LoadedImage = Image.open("img/Racetrack2D.png")#.resize((5*80, 80)) 
    LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
    racetrack2DImage = ImageTk.PhotoImage(LoadedImage)


    # RACETRACK 3D VIEW

    IconSize = (400, 400)

    LoadedImage = Image.open("img/Racetrack3D.png")#.resize((5*80, 80)) 
    LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
    racetrack3DImage = ImageTk.PhotoImage(LoadedImage)


    return [racetrack2DImage, racetrack3DImage]








