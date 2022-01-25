function dragElement(draggerId) {
    let draggerElement = $('#' + draggerId);

    let x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    const imageSize = $('#image').get(0).getBoundingClientRect();
    const draggerSize = draggerElement.get(0).getBoundingClientRect();

    draggerElement.css(
        {
            'top': (imageSize.top + (imageSize.bottom - imageSize.top) / 2) + "px",
            'left': (imageSize.left + (imageSize.right - imageSize.left) / 2) + "px"
        }
    );

    $('#' + draggerId + "-handle").get(0).onmousedown = dragMouseDown;
    registerApplyButton();

    function dragMouseDown(e) {
        e = e || window.event;
        e.preventDefault();
        // get the mouse cursor position at startup:
        x2 = e.clientX;
        y2 = e.clientY;
        document.onmouseup = closeDragElement;
        // call a function whenever the cursor moves:
        document.onmousemove = elementDrag;
        $('body').css('cursor', 'none');
    }

    function elementDrag(e) {
        e = e || window.event;
        e.preventDefault();
        // calculate the new cursor position:
        x1 = x2 - e.clientX;
        y1 = y2 - e.clientY;
        x2 = e.clientX;
        y2 = e.clientY;

        // set the element's new position:
        const top = (draggerElement.get(0).offsetTop - y1);
        if (top > imageSize.top && top < imageSize.bottom - (draggerSize.bottom - draggerSize.top))
            draggerElement.get(0).style.top = top + "px";

        const left = (draggerElement.get(0).offsetLeft - x1);
        if (left > imageSize.left && left < imageSize.right - (draggerSize.right - draggerSize.left))
            draggerElement.get(0).style.left = left + "px";
    }

    function closeDragElement() {
        /* stop moving when mouse button is released:*/
        document.onmouseup = null;
        document.onmousemove = null;
        $('body').css('cursor', '');
    }

    function registerApplyButton() {
        $('.apply-btn').on('click', function () {
            const btnSize = $(this).get(0).getBoundingClientRect();
            const dragItem = $('#' + draggerId);
            const draggerSize = dragItem.get(0).getBoundingClientRect();

            $(this).hide();
            $('.drag-btn').hide();

            dragItem.css(
                {
                    'background-color': "rgba(255, 255, 255, 0)",
                    'top': draggerSize.top + (btnSize.bottom - btnSize.top - 2) + 'px'
                });
        });
    }
}

$(document).ready(function () {
    $('#add-label').on('click', function () {
        addNewLabel();
    });
});