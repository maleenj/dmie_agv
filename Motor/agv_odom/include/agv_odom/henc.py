
encoder_wraps_x = 0
encoder_wraps_y = 0
# def handle_dat_encoder(encoder_x,encoder_y,x):

#     sign_x=0
#     sign_y=0
    
#     if encoder_x.size>1:  

#         diff=encoder_x[x]-encoder_x[x-1]

#         if abs(diff) > 16770000:
                
#             sign_x=1
#             changevalx=diff
#             encoder_x[x]=encoder_x[x]+changevalx
            
            
#         if sign_x==1:
        
#             encoder_x[x+1]=encoder_x[x+1]+changevalx
            
            
#         diff=encoder_y[x]-encoder_y[x-1]

#         if abs(diff) > 16770000:
                
#             sign_y=1
#             changevaly=diff
#             encoder_y[x]=encoder_y[x]-changevaly   
            
            
#         if sign_y==1:
        
#             encoder_y[x+1]=encoder_y[x+1]-changevaly
        
#     encoder_mean=(encoder_x[x]-encoder_y[x])/2
    
#     return encoder_mean

def handle_dat_encoder(encoder_x, encoder_y, x):
    global encoder_wraps_x, encoder_wraps_y

    if encoder_x.size > 1:

        # process x
        
        # calculate diff
        diff = encoder_x[x] - encoder_x[x-1]
        #print("current x : ", encoder_x[x], "previous x :", encoder_x[x-1], "diff : ", diff)

        # check wrap around
        if abs(diff) > 16770000:
            # if positive wrap around, increase
            if abs(encoder_x[x]) < abs(encoder_x[x-1]):
                #print("Positive")
                encoder_wraps_x = encoder_wraps_x + 1
            # else decrease
            else:
                #print("Negative")
                encoder_wraps_x = encoder_wraps_x - 1

        tmpX = (16777216*encoder_wraps_x) + encoder_x[x]
        #print("tmpX : ", tmpX)

        # process y

        #calculate diff
        diff = encoder_y[x] - encoder_y[x-1]
        #print("current y : ", encoder_y[x], "previous y :", encoder_y[x-1], "diff : ", diff)

        # check wrap around
        if abs(diff) > 16770000:
            #if positive wrap around, increase
            if abs(encoder_y[x] < abs(encoder_y[x-1])):
                #print("Positive")
                encoder_wraps_y = encoder_wraps_y + 1
            # else decrease
            else:
                #print("Negative")
                encoder_wraps_y = encoder_wraps_y - 1

        tmpY = (16777216*encoder_wraps_y) + encoder_y[x]
        #print("tmpY : ", tmpY)

        encoder_mean=(tmpX-tmpY)/-2

    else:
        encoder_mean=(encoder_x[x]-encoder_y[x])/-2

    #print("encoder mean : ", encoder_mean)

    return encoder_mean
