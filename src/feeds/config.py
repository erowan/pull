"""
Example Config
"""

__doc__="""
   Config is used by pull.site to dynamically load feeds.
   """
Config = {
    # module name      
    'feeds.health' : {
        # feed_name : feed_class                
        'who'  : 'WhoOutbreaks',  
        'food' : 'FoodOutbreaks'     
    },
    
}
