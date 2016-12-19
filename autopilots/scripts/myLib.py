#####
# myMathLib.py
#
# z = sat(x,Xl,Xu)
# z = dead(x,X)
# class xyVar()
#
#####

def sat(x,Xl,Xu):
    z = x
    if x > Xu:
        z = Xu
    elif x < Xl:
        z = Xl
    return z


def dead(x,X):
    z = x - sat(x,-X,X)
    return z
    
    
class xyVar:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0



 
