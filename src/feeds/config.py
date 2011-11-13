"""
Example Config
"""

__doc__="""
   Config is a dict key'd on feeder name which contains as it's value a
   dict of key=feed module name, value=feed_class_name."""
Config = {
    'feeds.health' : {
        'who'  : 'WhoOutbreaks',  
        'food' : 'FoodOutbreaks'     
    },
    
}
