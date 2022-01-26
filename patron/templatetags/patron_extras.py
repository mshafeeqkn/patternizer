import html

from django import template

register = template.Library()


@register.filter(name='lookup')
def lookup(input_dict, args):
    return input_dict.get(args, '')
